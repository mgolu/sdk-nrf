/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#define DT_DRV_COMPAT nordic_nrf_slm

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_nrf_slm, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <fcntl.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include <modem_context.h>
#include <modem_socket.h>
#include <modem_cmd_handler.h>
#include <modem_iface_uart.h>

#define MDM_UART_NODE			DT_INST_BUS(0)
#define MDM_UART_DEV			DEVICE_DT_GET(MDM_UART_NODE)

#define MDM_RESET_NOT_ASSERTED		1
#define MDM_RESET_ASSERTED		0

#define MDM_CMD_TIMEOUT			K_SECONDS(10)
#define MDM_DNS_TIMEOUT			K_SECONDS(70)
#define MDM_CMD_CONN_TIMEOUT		K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT	K_SECONDS(180)
#define MDM_PROMPT_CMD_DELAY		K_MSEC(50)
#define RSSI_TIMEOUT           K_SECONDS(30)

#define MDM_MAX_DATA_LENGTH		1024
#define MDM_RECV_MAX_BUF		30
#define MDM_RECV_BUF_SIZE		128

#define CONNECT_HOSTNAME_LENGTH	128

#define MDM_MAX_SOCKETS			6
#define MDM_BASE_SOCKET_NUM		0

#define MDM_NETWORK_RETRY_COUNT		3
#define MDM_WAIT_FOR_RSSI_COUNT		10
#define MDM_WAIT_FOR_RSSI_DELAY		K_SECONDS(2)

#define MDM_MANUFACTURER_LENGTH		21
#define MDM_MODEL_LENGTH		16
#define MDM_REVISION_LENGTH		64
#define MDM_IMEI_LENGTH			16
#define MDM_IMSI_LENGTH			16
#define MDM_APN_LENGTH			32
#define MDM_MAX_CERT_LENGTH		8192

/* driver data */
struct modem_data {
	struct net_if *net_iface;
	uint8_t mac_addr[6];

	/* modem interface */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* modem cmds */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* RSSI work */
	struct k_work_delayable rssi_query_work;

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
	char mdm_imsi[MDM_IMSI_LENGTH];
	int mdm_rssi;

	/* modem state */
	int ev_creg;

	/* bytes written to socket in last transaction */
	int sock_written;

	/* response semaphore */
	struct k_sem sem_response;

	/* datamode state*/
	bool datamode;

	/* Socket ID created */
	int sock_id;
};

static struct modem_data mdata;

NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE,
		    0, NULL);

/* RX thread structures */
K_KERNEL_STACK_DEFINE(modem_rx_stack,
		      CONFIG_MODEM_NRF_SLM_RX_STACK_SIZE);
struct k_thread modem_rx_thread;

/* RX thread work queue */
K_KERNEL_STACK_DEFINE(modem_workq_stack,
		      CONFIG_MODEM_NRF_SLM_RX_WORKQ_STACK_SIZE);
static struct k_work_q modem_workq;

/* socket read callback data */
struct socket_read_data {
	char *recv_buf;
	size_t recv_buf_len;
	struct sockaddr *recv_addr;
	uint16_t recv_read_len;
};

static struct modem_context mctx;

int selected_socket;

/* helper macro to keep readability */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

/**
 * @brief  Convert string to long integer, but handle errors
 *
 * @param  s: string with representation of integer number
 * @param  err_value: on error return this value instead
 * @param  desc: name the string being converted
 * @param  func: function where this is called (typically __func__)
 *
 * @retval return integer conversion on success, or err_value on error
 */
static int modem_atoi(const char *s, const int err_value,
		      const char *desc, const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", s, desc,
			func);
		return err_value;
	}

	return ret;
}

/* Forward declaration */
MODEM_CMD_DEFINE(on_cmd_sockwrite);
static int switch_socket_to_handle(int id);

/* send binary data via the #XSEND command in data mode */
static ssize_t send_socket_data(void *obj,
				const struct msghdr *msg,
				k_timeout_t timeout)
{
	int ret;
	char send_buf[sizeof("AT#XSENDTO=\r\n")+256]; // TODO: Fix the buffer size
	uint16_t dst_port = 0U;
	struct modem_socket *sock = (struct modem_socket *)obj;

	const struct modem_cmd handler_cmds[] = {
		MODEM_CMD("#XDATAMODE: ", on_cmd_sockwrite, 1U, ""),
	};
	struct sockaddr *dst_addr = msg->msg_name;
	size_t buf_len = 0;

	if (!sock) {
		return -EINVAL;
	}

	for (int i = 0; i < msg->msg_iovlen; i++) {
		if (!msg->msg_iov[i].iov_base || msg->msg_iov[i].iov_len == 0) {
			errno = EINVAL;
			return -1;
		}
		buf_len += msg->msg_iov[i].iov_len;
	}

	if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP) {
		errno = ENOTCONN;
		return -1;
	}

	if (!dst_addr && sock->ip_proto == IPPROTO_UDP) {
		dst_addr = &sock->dst;
	}

	/* The number of bytes written will be reported by the modem */
	mdata.sock_written = 0;

	if (sock->ip_proto == IPPROTO_UDP) {
		char ip_str[NET_IPV6_ADDR_LEN];

		ret = modem_context_sprint_ip_addr(dst_addr, ip_str, sizeof(ip_str));
		if (ret != 0) {
			LOG_ERR("Error formatting IP string %d", ret);
			goto exit;
		}

		ret = modem_context_get_addr_port(dst_addr, &dst_port);
		if (ret != 0) {
			LOG_ERR("Error getting port from IP address %d", ret);
			goto exit;
		}

		snprintk(send_buf, sizeof(send_buf),
			 "AT#XSENDTO=\"%s\",%u", ip_str, dst_port);
	} else {
		snprintk(send_buf, sizeof(send_buf), "AT#XSEND");
	}

	switch_socket_to_handle(sock->id);

	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);

	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler,
				    NULL, 0U, send_buf, &mdata.sem_response, 
					MDM_CMD_TIMEOUT);
	if (ret < 0) {
		goto exit;
	}
	mdata.datamode = true;
	
	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    handler_cmds,
					    ARRAY_SIZE(handler_cmds),
					    true);
	if (ret < 0) {
		goto exit;
	}
	modem_cmd_handler_set_error(mctx.cmd_handler.cmd_handler_data, 0);

	/* Reset response semaphore before sending data
	 * So that we are sure that we won't use a previously pending one
	 * And we won't miss the one that is going to be freed
	 */
	k_sem_reset(&mdata.sem_response);

	/* Send data directly on modem iface */
	for (int i = 0; i < msg->msg_iovlen; i++) {
		int len = MIN(buf_len, msg->msg_iov[i].iov_len);

		if (len == 0) {
			break;
		}
		mctx.iface.write(&mctx.iface, msg->msg_iov[i].iov_base, len);
		buf_len -= len;
	}
terminate:
	int ts = snprintk(send_buf, sizeof(send_buf), CONFIG_SLM_DATAMODE_TERMINATOR);
	mctx.iface.write(&mctx.iface, send_buf, MIN(ts, strlen(CONFIG_SLM_DATAMODE_TERMINATOR)));
	ret = k_sem_take(&mdata.sem_response, K_FOREVER);

	if (ret == 0) {
		ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	} else if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
	}

exit:
	if (mdata.datamode) {
		goto terminate;
	}
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    NULL, 0U, false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	if (ret < 0) {
		return ret;
	}

	return mdata.sock_written;
}

/*
 * Modem Response Command Handlers
 */

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME Error: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * Modem Info Command Handlers
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_manufacturer,
				    sizeof(mdata.mdm_manufacturer) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_model,
				    sizeof(mdata.mdm_model) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", mdata.mdm_model);

	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_revision,
				    sizeof(mdata.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", mdata.mdm_revision);
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_imei, sizeof(mdata.mdm_imei) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", mdata.mdm_imei);
	return 0;
}

/* Handler: <IMSI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imsi)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_imsi, sizeof(mdata.mdm_imsi) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';
	LOG_INF("IMSI: %s", mdata.mdm_imsi);

#if defined(CONFIG_MODEM_UBLOX_SARA_AUTODETECT_APN)
	/* set the APN automatically */
	modem_detect_apn(mdata.mdm_imsi);
#endif

	return 0;
}

/*
 * Handler: +CESQ: <rxlev>[0],<ber>[1],<rscp>[2],<ecn0>[3],<rsrq>[4],<rsrp>[5]
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_cesq)
{
	int rsrp, rxlev;

	rsrp = ATOI(argv[5], 0, "rsrp");
	rxlev = ATOI(argv[0], 0, "rxlev");
	if (rsrp >= 0 && rsrp <= 97) {
		mdata.mdm_rssi = -140 + (rsrp - 1);
		LOG_INF("RSRP: %d", mdata.mdm_rssi);
	} else if (rxlev >= 0 && rxlev <= 63) {
		mdata.mdm_rssi = -110 + (rxlev - 1);
		LOG_INF("RSSI: %d", mdata.mdm_rssi);
	} else {
		mdata.mdm_rssi = -1000;
		LOG_INF("RSRP/RSSI not known");
	}

	return 0;
}

static int unquoted_atoi(const char *s, int base)
{
	if (*s == '"') {
		s++;
	}

	return strtol(s, NULL, base);
}

/*
 * Modem Socket Command Handlers
 */

/* Handler: #XSOCKET: <handle>,<type>,<protocol> */
MODEM_CMD_DEFINE(on_cmd_sockcreate)
{
	mdata.sock_id = 
		ATOI(argv[0], mdata.socket_config.base_socket_num - 1,
				"socket_id");

	/* don't give back semaphore -- OK to follow */
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_sockclose)
{
	struct modem_socket *sock = modem_socket_from_id(
			&mdata.socket_config, selected_socket);
	if (sock && ATOI(argv[0], 0, "closed") == 0) {
		sock->is_connected = false;
	}

	return 0;
}

MODEM_CMD_DEFINE(on_cmd_sockselect)
{
	int ret = ATOI(argv[0], 
			mdata.socket_config.base_socket_num - 1, 
			"socket_id");
	if (ret < 0) {
		LOG_ERR("#XSOCKETSELECT error");
		return -EINVAL;
	}	
	selected_socket = ret;
	return 0;
}

/* Handler: #XDATAMODE: <length> */
MODEM_CMD_DEFINE(on_cmd_sockwrite)
{
	int ret = ATOI(argv[0], 0, "length");
	if (ret == 0) {
		mdata.datamode = false;
		k_sem_give(&mdata.sem_response);
		LOG_DBG("Exiting datamode");
	} else if (ret < 0) {
		modem_cmd_handler_set_error(data, ret);
	} else if (ret > 0) {
		mdata.sock_written = ret;
		LOG_DBG("bytes written: %d", mdata.sock_written);
	}
	return 0;
}

static bool is_crlf(uint8_t c)
{
	if (c == '\n' || c == '\r') {
		return true;
	} else {
		return false;
	}
}

static uint16_t findcrlf(struct modem_cmd_handler_data *data,
		      struct net_buf **frag, uint16_t *offset)
{
	struct net_buf *buf = data->rx_buf;
	uint16_t len = 0U, pos = 0U;

	while (buf && buf->len && !is_crlf(*(buf->data + pos))) {
		if (pos + 1 >= buf->len) {
			len += buf->len;
			buf = buf->frags;
			pos = 0U;
		} else {
			pos++;
		}
	}

	if (buf && buf->len && is_crlf(*(buf->data + pos))) {
		len += pos;
		*offset = pos;
		*frag = buf;
		return len;
	}

	return 0;
}

/*
 * Handler: #XRECVFROM: length,"<remote_ip_addr>"\r\n<data>
 * Handler: #XRECV: length\r\n<data>
*/
MODEM_CMD_DIRECT_DEFINE(on_cmd_sockreadfrom)
{
	struct net_buf *frag = NULL;
	struct modem_socket *sock = NULL;
	struct socket_read_data *sock_data;
	int ret;
	size_t match_len, begin, end;
	uint16_t offset, recv_len = 0;

	len = findcrlf(data, &frag, &offset);

	if (!len) {
		/* No CRLF, wait for more data */
		return -EAGAIN;
	}

	/* load match_buf with the returned command line, without the data */
	match_len = net_buf_linearize(data->match_buf, data->match_buf_len - 1,
			data->rx_buf, 0, len);
		
	k_sem_take(&data->sem_parse_lock, K_FOREVER);

	if (strncmp(data->match_buf, "#XRECVFROM:", strlen("#XRECVFROM:")) == 0) {
		begin = strlen("#XRECVFROM:");
		end = strlen("#XRECVFROM:");
	} else {
		begin = strlen("#XRECV:");
		end = strlen("#XRECV:");
	}
	while (end < match_len) {
		if (data->match_buf[end] == ',' || data->match_buf[end] == '\r') {
			data->match_buf[end] = '\0';
			recv_len = ATOI(&data->match_buf[begin], 0, "recv_len");
			break;
		}
		end++;
	}

	if (data->rx_buf->len < len + 2 + recv_len) {
		/* Not enough data received, wait for more */
		k_sem_give(&data->sem_parse_lock);
		return -EAGAIN;
	}

	if (data->match_buf[end + 1] == '\"') {
		/* TODO: Handle getting source IP address. For now, just skip to data */
		data->rx_buf = net_buf_skip(data->rx_buf, len + 2);
	} else {
		/* Skip the previously detected line + 2 for the \r\n */
		data->rx_buf = net_buf_skip(data->rx_buf, len + 2);
	}
	k_sem_give(&data->sem_parse_lock);

	sock = modem_socket_from_id(&mdata.socket_config, selected_socket);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", selected_socket);
		ret = -EINVAL;
		goto exit;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("Socket data not found! Skip handling (%d)", selected_socket);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len,
				data->rx_buf, 0, recv_len);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;
	if (ret != recv_len) {
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d", ret, recv_len);
		ret = -EINVAL;
	}

exit:
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock, 0);
	/* don't give back semaphore -- OK to follow */
	return ret;
}

/* Handler: #XGETADDRINFO: "<resolved_ip_address> <etc> <etc>" */
MODEM_CMD_DEFINE(on_cmd_getaddrinfo)
{
	char *addr, *rest;
	const char sep[2] = " ";
	struct zsock_addrinfo* ai_current = NULL;

	if (strlen(argv[0]) < 1) {
		return -1;
	}
	/* Avoid a memory leak */
	if (data->user_data != NULL) {
		LOG_DBG("User data is not null");
		return -1;
	}
	/* Remove trailing quote */
	argv[0][strlen(argv[0]) - 1] = '\0';

	addr = strtok_r(argv[0]+1, sep, &rest);

	while (addr != NULL) {
		LOG_DBG("DNS address: %s", addr);
		if (data->user_data == NULL) {
			data->user_data = malloc(sizeof(struct zsock_addrinfo));
			ai_current = (struct zsock_addrinfo *)(data->user_data);
		} else {
			ai_current->ai_next = malloc(sizeof(struct zsock_addrinfo));
			ai_current = ai_current->ai_next;
		}
		ai_current->ai_family = AF_INET;
		if (strchr(addr, ':') != NULL) {
			ai_current->ai_family = AF_INET6;
		} 
		ai_current->ai_addr = malloc(sizeof(struct sockaddr));
		ai_current->ai_addr->sa_family = ai_current->ai_family;
		if (ai_current->ai_family == AF_INET) {
			inet_pton(AF_INET, addr, &((struct sockaddr_in *)ai_current->ai_addr)->sin_addr);
		} else if (ai_current->ai_family == AF_INET6) {
			inet_pton(AF_INET6, addr, &((struct sockaddr_in6 *)ai_current->ai_addr)->sin6_addr);
		}
		addr = strtok_r(rest, sep, &rest);
	}

	return 0;
}

/*
 * MODEM UNSOLICITED NOTIFICATION HANDLERS
 */

/* Handler: #XPOLL: <socket_id>[0],<revents>[1] */
MODEM_CMD_DEFINE(on_cmd_socknotifydata)
{
	int socket_id, revents;
	struct modem_socket *sock;

	if (argc < 2) {
		LOG_ERR("Socket poll error");
		return 0;
	}

	socket_id = ATOI(argv[0], 0, "socket_id");
	revents = unquoted_atoi(argv[1], 0);
	sock = modem_socket_from_id(&mdata.socket_config, socket_id);
	if (!sock) {
		return 0;
	}

	if (revents & ZSOCK_POLLERR) {
		return -1;
	}

	if (revents & ZSOCK_POLLIN) {
		/* We don't know how much data, so put a dummy value to set revents */
		modem_socket_packet_size_update(&mdata.socket_config, sock, 1);
		modem_socket_data_ready(&mdata.socket_config, sock);
	}

	return 0;
}

/* Handler: +CEREG: <stat>[0] */
MODEM_CMD_DEFINE(on_cmd_socknotifycereg)
{
	uint8_t stat_index;
	stat_index = (argc > 1) ? 1 : 0;
	mdata.ev_creg = ATOI(argv[stat_index], 0, "stat");
	LOG_DBG("CEREG: %d", mdata.ev_creg);
	return 0;
}

/* RX thread */
static void modem_rx(void)
{
	while (true) {
		/* wait for incoming data */
		k_sem_take(&mdata.iface_data.rx_sem, K_FOREVER);

		mctx.cmd_handler.process(&mctx.cmd_handler, &mctx.iface);

		/* give up time if we have a solid stream of data */
		k_yield();
	}
}

static int pin_init(void)
{
	LOG_INF("Setting Modem Pins");

	LOG_INF("... Done!");

	return 0;
}

static void modem_rssi_query_work(struct k_work *work)
{
	static const struct modem_cmd cmd =
		MODEM_CMD("+CESQ: ", on_cmd_atcmdinfo_rssi_cesq, 6U, ",");
	static char *send_cmd = "AT+CESQ";
	int ret;

	/* query modem RSSI */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     &cmd, 1U, send_cmd, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CESQ ret:%d", ret);
	}

	/* re-start RSSI query work */
	if (work) {
		k_work_reschedule_for_queue(
			&modem_workq, &mdata.rssi_query_work,
			RSSI_TIMEOUT);
	}
}

static void modem_reset(void)
{
	int ret = 0, retry_count = 0, counter = 0;
	static const struct setup_cmd setup_cmds[] = {
		/* Set to airplane mode */
		SETUP_CMD_NOHANDLE("AT+CFUN=4"),
		/* query modem info */
		SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
		SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
		SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
		SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),
        /* extended error numbers */
		SETUP_CMD_NOHANDLE("AT+CMEE=1"),
        SETUP_CMD_NOHANDLE("AT+CEREG=1"),
        SETUP_CMD_NOHANDLE("AT+COPS=0"),
        SETUP_CMD_NOHANDLE("AT+CFUN=1"),
		SETUP_CMD("AT+CIMI", "", on_cmd_atcmdinfo_imsi, 0U, ""),
	};

restart:

	/* stop RSSI delay work */
	k_work_cancel_delayable(&mdata.rssi_query_work);

	pin_init();

	LOG_INF("Waiting for modem to respond");

	/* Give the modem a while to start responding to simple 'AT' commands.
	 * Also wait for CSPS=1 or RRCSTATE=1 notification
	 */
	ret = -1;
	while (counter++ < 50 && ret < 0) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0, "AT", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret < 0 && ret != -ETIMEDOUT) {
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("MODEM WAIT LOOP ERROR: %d", ret);
		goto error;
	}

	char sendbuf[sizeof("AT+CEREG?\r\n") + 128];

	snprintk(sendbuf, sizeof(sendbuf), "AT+CEREG?");
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0, sendbuf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);

	if (mdata.ev_creg == 1 || mdata.ev_creg == 5) {
		goto initialized;
	}

	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds, ARRAY_SIZE(setup_cmds),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		goto error;
	}

	LOG_INF("Waiting for network");

	/*
	 * TODO: A lot of this should be setup as a 3GPP module to handle
	 * basic connection to the network commands / polling
	 */

	/* wait for +CEREG: 1(normal) or 5(roaming) */
	counter = 0;
	while (counter++ < 40 && mdata.ev_creg != 1 && mdata.ev_creg != 5) {
		if (counter == 20) {
			LOG_WRN("Force restart of RF functionality");

			/* Disable RF temporarily */
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				NULL, 0, "AT+CFUN=4", &mdata.sem_response,
				MDM_CMD_TIMEOUT);

			k_sleep(K_SECONDS(1));

			/* Enable RF */
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				NULL, 0, "AT+CFUN=1", &mdata.sem_response,
				MDM_CMD_TIMEOUT);
		}

		k_sleep(K_SECONDS(1));
	}

	/* query modem RSSI */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	counter = 0;
	/* wait for RSSI < 0 and > -1000 */
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mdata.mdm_rssi >= 0 ||
		mdata.mdm_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		retry_count++;
		if (retry_count >= MDM_NETWORK_RETRY_COUNT) {
			LOG_ERR("Failed network init.  Too many attempts!");
			ret = -ENETUNREACH;
			goto error;
		}

		LOG_ERR("Failed network init.  Restarting process.");
		goto restart;
	}

initialized:
	LOG_INF("Network is ready.");

	/* start RSSI query */
	k_work_reschedule_for_queue(
		&modem_workq, &mdata.rssi_query_work,
		RSSI_TIMEOUT);

error:
	return;
}

/*
 * generic socket creation function
 * which can be called in bind() or connect()
 */
static int create_socket(struct modem_socket *sock, const struct sockaddr *addr, bool server)
{
	int ret;
	static const struct modem_cmd cmd =
		MODEM_CMD("#XSOCKET: ", on_cmd_sockcreate, 3U, ",");
	char buf[sizeof("AT#XSOCKET=#,#,#,#\r\n")];
	uint16_t op = 1U, type = 1U;

	if (addr->sa_family == AF_INET6) {
		op = 2U;
	}

	if (sock->ip_proto == IPPROTO_UDP) {
		type = 2U;
	}

	snprintk(buf, sizeof(buf), "AT#XSOCKET=%u,%u,%u", op, type, server ? 1U : 0U);

	/* create socket */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     &cmd, 1U, buf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		goto error;
	}
	sock->id = mdata.sock_id;
	memcpy(&sock->dst, addr, sizeof(struct sockaddr));

	errno = 0;
	return 0;

error:
	LOG_ERR("%s ret:%d", buf, ret);
	modem_socket_put(&mdata.socket_config, sock->sock_fd);
	errno = -ret;
	return -1;
}

/*
 * Switch the active socket in SLM by the socket handle
 */
static int switch_socket_to_handle(int id)
{
	int ret;
	static const struct modem_cmd cmd =
		MODEM_CMD("#XSOCKETSELECT: ", on_cmd_sockselect, 1U, "");
	char buf[sizeof("AT#XSOCKETSELECT=##\r\n")];

	snprintk(buf, sizeof(buf), "AT#XSOCKETSELECT=%d", id);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				&cmd, 1U, buf,
				&mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		goto error;
	}

	errno = 0;
	return 0;
error:
	LOG_ERR("%s ret:%d", buf, ret);
	errno = -ret;
	return -1;
}

/*
 * Socket Offload OPS
 */

static const struct socket_op_vtable offload_socket_fd_op_vtable;

static int offload_socket(int family, int type, int proto)
{
	int ret;

	/* defer modem's socket create call to bind() or connect() */
	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	int ret;

	static const struct modem_cmd cmd =
		MODEM_CMD("#XSOCKET: ", on_cmd_sockclose, 2U, ",");

	/* make sure we assigned an id */
	if (sock->id < mdata.socket_config.base_socket_num) {
		return 0;
	}

	if (sock->is_connected || sock->ip_proto == IPPROTO_UDP) {
		switch_socket_to_handle(sock->id);

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     &cmd, 1U, "AT#XSOCKET=0",
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%s ret:%d", "AT#XSOCKET=0", ret);
		}
	}

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
	return 0;
}

static int offload_bind(void *obj, const struct sockaddr *addr,
			socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* save bind address information */
	memcpy(&sock->src, addr, sizeof(*addr));

	LOG_DBG("Socket bind %d", sock->id);

	if (create_socket(sock, addr, true) < 0) {
		return -1;
	}

	char buf[sizeof("AT#XBIND=#####\r\n")];
	uint16_t local_port = 0U;
	int ret;

	if (addr) {
		if (addr->sa_family == AF_INET6) {
			local_port = ntohs(net_sin6(addr)->sin6_port);
		} else if (addr->sa_family == AF_INET) {
			local_port = ntohs(net_sin(addr)->sin_port);
		}
	}

	if (local_port > 0U) {
		snprintk(buf, sizeof(buf), "AT#XBIND=%u", local_port);
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				NULL, 0U, buf,
				&mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("%s ret:%d", buf, ret);
		}
	}

	return 0;
}

static int offload_connect(void *obj, const struct sockaddr *addr,
			   socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	int ret;
	char buf[sizeof("AT+XCONNECT=,#####\r\n") + CONNECT_HOSTNAME_LENGTH];
	uint16_t dst_port = 0U;
	char ip_str[NET_IPV6_ADDR_LEN];

	if (!addr) {
		errno = EINVAL;
		return -1;
	}

	if (sock->id < mdata.socket_config.base_socket_num - 1) {
		LOG_ERR("Invalid socket_id(%d) from fd:%d",
			sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (create_socket(sock, addr, false) < 0) {
		return -1;
	}

	memcpy(&sock->dst, addr, sizeof(*addr));
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	} else {
		errno = EAFNOSUPPORT;
		return -1;
	}

	/* skip socket connect if UDP */
	if (sock->ip_proto == IPPROTO_UDP) {
		errno = 0;
		return 0;
	}

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		errno = -ret;
		LOG_ERR("Error formatting IP string %d", ret);
		return -1;
	}

	snprintk(buf, sizeof(buf), "AT+XCONNECT=\"%s\",%d", ip_str, dst_port);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U, buf,
			     &mdata.sem_response, MDM_CMD_CONN_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", buf, ret);
		errno = -ret;
		return -1;
	}

	sock->is_connected = true;
	errno = 0;
	return 0;
}

static ssize_t offload_recvfrom(void *obj, void *buf, size_t len,
				int flags, struct sockaddr *from,
				socklen_t *fromlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	int ret, packet_size;

	static const struct modem_cmd cmd[] = {
		MODEM_CMD_DIRECT("#XRECVFROM: ", on_cmd_sockreadfrom),
	};
	char sendbuf[sizeof("AT#XRECVFROM=######\r")];
	struct socket_read_data sock_data;

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	/* This will return 0 if XPOLL hasn't returned yet, otherwise 1, since we
		don't get the data size from XPOLL response */
	packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (!packet_size) {
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}

		modem_socket_wait_data(&mdata.socket_config, sock);
		packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT#XRECV%s=%d",
		 sock->ip_proto == IPPROTO_UDP ? "FROM" : "", 20); // TODO: Fix timeout

	/* socket read settings */
	(void)memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf = buf;
	sock_data.recv_buf_len = len;
	sock_data.recv_addr = from;
	sock->data = &sock_data;

	switch_socket_to_handle(sock->id);

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmd, ARRAY_SIZE(cmd), sendbuf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto exit;
	}

	/* HACK: use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	/* return length of received data */
	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	/* clear socket data */
	sock->data = NULL;
	return ret;
}

static ssize_t offload_sendto(void *obj, const void *buf, size_t len,
			      int flags, const struct sockaddr *to,
			      socklen_t tolen)
{
	struct iovec msg_iov = {
		.iov_base = (void *)buf,
		.iov_len = len,
	};
	struct msghdr msg = {
		.msg_iovlen = 1,
		.msg_name = (struct sockaddr *)to,
		.msg_namelen = tolen,
		.msg_iov = &msg_iov,
	};

	int ret = send_socket_data(obj, &msg, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	errno = 0;
	return ret;
}

static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	char sendbuf[sizeof("AT#XPOLL=#######,##,##,##,##,##,##,##,##\r\n")];

	const struct modem_cmd cmd =
		MODEM_CMD_ARGS_MAX("#XPOLL: ", on_cmd_socknotifydata, 1U, 2U, ",");

	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		return -EXDEV;
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		return -EOPNOTSUPP;
	}

	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zsock_pollfd *fds;
		int nfds, timeout;

		fds = va_arg(args, struct zsock_pollfd *);
		nfds = va_arg(args, int);
		timeout = va_arg(args, int);

		snprintk(sendbuf, sizeof(sendbuf), "AT#XPOLL=%d,%u", timeout,
						((struct modem_socket *)obj)->id);

		modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				&cmd, 1U, sendbuf, &mdata.sem_response,
				K_MSEC(timeout));

		return modem_socket_poll(&mdata.socket_config, fds, nfds, timeout);
	}

	case F_GETFL:
		return 0;

	default:
		errno = EINVAL;
		return -1;
	}
}

static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int bkp_iovec_idx;
	struct iovec bkp_iovec = {0};
	struct msghdr crafted_msg = {
		.msg_name = msg->msg_name,
		.msg_namelen = msg->msg_namelen,
	};
	size_t full_len = 0;
	int ret;

	/* Compute the full length to be send and check for invalid values */
	for (int i = 0; i < msg->msg_iovlen; i++) {
		if (!msg->msg_iov[i].iov_base || msg->msg_iov[i].iov_len == 0) {
			errno = EINVAL;
			return -1;
		}
		full_len += msg->msg_iov[i].iov_len;
	}

	LOG_DBG("msg_iovlen:%zd flags:%d, full_len:%zd",
		msg->msg_iovlen, flags, full_len);

	while (full_len > sent) {
		int removed = 0;
		int i = 0;

		crafted_msg.msg_iovlen = msg->msg_iovlen;
		crafted_msg.msg_iov = &msg->msg_iov[0];

		bkp_iovec_idx = -1;
		/*  Iterate on iovec to remove the bytes already sent */
		while (removed < sent) {
			int to_removed = sent - removed;

			if (to_removed >= msg->msg_iov[i].iov_len) {
				crafted_msg.msg_iovlen -= 1;
				crafted_msg.msg_iov = &msg->msg_iov[i + 1];

				removed += msg->msg_iov[i].iov_len;
			} else {
				/* Backup msg->msg_iov[i] before "removing"
				 * starting bytes already send.
				 */
				bkp_iovec_idx = i;
				bkp_iovec.iov_len = msg->msg_iov[i].iov_len;
				bkp_iovec.iov_base = msg->msg_iov[i].iov_base;

				/* Update msg->msg_iov[i] to "remove"
				 * starting bytes already send.
				 */
				msg->msg_iov[i].iov_len -= to_removed;
				msg->msg_iov[i].iov_base = &(((uint8_t *)msg->msg_iov[i].iov_base)[to_removed]);

				removed += to_removed;
			}

			i++;
		}

		ret = send_socket_data(obj, &crafted_msg, MDM_CMD_TIMEOUT);

		/* Restore backup iovec when necessary */
		if (bkp_iovec_idx != -1) {
			msg->msg_iov[bkp_iovec_idx].iov_len = bkp_iovec.iov_len;
			msg->msg_iov[bkp_iovec_idx].iov_base = bkp_iovec.iov_base;
		}

		/* Handle send_socket_data() returned value */
		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		sent += ret;
	}

	return (ssize_t)sent;
}

static int offload_setsockopt(void *obj, int level, int optname,
			      const void *optval, socklen_t optlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	int ret;

	if (IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS) && level == SOL_TLS) {
		switch (optname) {
		case TLS_SEC_TAG_LIST:
			return -EINVAL;
			break;
		case TLS_HOSTNAME:
			LOG_WRN("TLS_HOSTNAME option is not supported");
			return -EINVAL;
		case TLS_PEER_VERIFY:
			if (*(uint32_t *)optval != TLS_PEER_VERIFY_REQUIRED) {
				LOG_WRN("Disabling peer verification is not supported");
				return -EINVAL;
			}
			ret = 0;
			break;
		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return ret;
}


static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read = offload_read,
		.write = offload_write,
		.close = offload_close,
		.ioctl = offload_ioctl,
	},
	.bind = offload_bind,
	.connect = offload_connect,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.listen = NULL,
	.accept = NULL,
	.sendmsg = offload_sendmsg,
	.getsockopt = NULL,
	.setsockopt = offload_setsockopt,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET &&
	    family != AF_INET6) {
		return false;
	}

	if (type != SOCK_DGRAM &&
	    type != SOCK_STREAM) {
		return false;
	}

	if (proto != IPPROTO_TCP &&
	    proto != IPPROTO_UDP &&
	    proto != IPPROTO_TLS_1_2) {
		return false;
	}

	return true;
}

NET_SOCKET_OFFLOAD_REGISTER(nrf_slm, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC, offload_is_supported, offload_socket);

static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints,
			       struct zsock_addrinfo **res)
{
	static const struct modem_cmd cmd =
		MODEM_CMD("#XGETADDRINFO: ", on_cmd_getaddrinfo, 1U, "");
	uint32_t port = 0U;
	int ret;
	/* DNS command + 128 bytes for domain name parameter */
	char sendbuf[sizeof("AT#XGETADDRINFO=##\r\n") + 128];

	if (service) {
		port = ATOI(service, 0U, "port");
		if (port < 1 || port > USHRT_MAX) {
			return DNS_EAI_SERVICE;
		}
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT#XGETADDRINFO=\"%s\"", node);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     &cmd, 1U, sendbuf, &mdata.sem_response,
			     MDM_DNS_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	*res = (struct zsock_addrinfo *)mdata.cmd_handler_data.user_data;
	return 0;
}

static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	struct zsock_addrinfo* ai = res;

	while (res != NULL) {
		ai = res;
		res = res->ai_next;
		free(ai);
	}
}

const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};

static int net_offload_dummy_get(sa_family_t family,
				 enum net_sock_type type,
				 enum net_ip_protocol ip_proto,
				 struct net_context **context)
{

	LOG_ERR("CONFIG_NET_SOCKETS_OFFLOAD must be enabled for this driver");

	return -ENOTSUP;
}

/* placeholders, until Zephyr IP stack updated to handle a NULL net_offload */
static struct net_offload modem_net_offload = {
	.get = net_offload_dummy_get,
};

#define HASH_MULTIPLIER		37
static uint32_t hash32(char *str, int len)
{
	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

static int offload_socket(int family, int type, int proto);

static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data = dev->data;

	/* Direct socket offload used instead of net offload: */
	iface->if_dev->offload = &modem_net_offload;
	net_if_set_link_addr(iface, modem_get_mac(dev),
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;

#ifdef CONFIG_DNS_RESOLVER
	socket_offload_dns_register(&offload_dns_ops);
#endif

	net_if_socket_offload_set(iface, offload_socket);
}

static struct net_if_api api_funcs = {
	.init = modem_net_iface_init,
};

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""), /* 3GPP */
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""), /* 3GPP */
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD_ARGS_MAX("+CEREG: ", on_cmd_socknotifycereg, 1U, 8U, ","),
};

static int modem_init(const struct device *dev)
{
    int ret = 0;

    ARG_UNUSED(dev);
	mdata.datamode = false;

    k_sem_init(&mdata.sem_response, 0, 1);

    /* socket config */
	mdata.socket_config.sockets = &mdata.sockets[0];
	mdata.socket_config.sockets_len = ARRAY_SIZE(mdata.sockets);
	mdata.socket_config.base_socket_num = MDM_BASE_SOCKET_NUM;
	ret = modem_socket_init(&mdata.socket_config,
				&offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

    /* cmd handler */
	mdata.cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	mdata.cmd_handler_data.cmds[CMD_UNSOL] = unsol_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsol_cmds);
	mdata.cmd_handler_data.match_buf = &mdata.cmd_match_buf[0];
	mdata.cmd_handler_data.match_buf_len = sizeof(mdata.cmd_match_buf);
	mdata.cmd_handler_data.buf_pool = &mdm_recv_pool;
	mdata.cmd_handler_data.alloc_timeout = K_NO_WAIT;
	mdata.cmd_handler_data.eol = "\r\n";
	ret = modem_cmd_handler_init(&mctx.cmd_handler,
				     &mdata.cmd_handler_data);
	if (ret < 0) {
		goto error;
	}

	/* modem interface */
	mdata.iface_data.hw_flow_control = DT_PROP(MDM_UART_NODE,
						   hw_flow_control);
	mdata.iface_data.rx_rb_buf = &mdata.iface_rb_buf[0];
	mdata.iface_data.rx_rb_buf_len = sizeof(mdata.iface_rb_buf);
	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data,
				    MDM_UART_DEV);
	if (ret < 0) {
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;
	mctx.data_rssi = &mdata.mdm_rssi;

    mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
			K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			(k_thread_entry_t) modem_rx,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	if (mdata.ev_creg != 1 && mdata.ev_creg != 5) {
		modem_reset();
	}

error:
    return ret;
}


/* Register device with the networking stack. */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_NRF_SLM_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);
