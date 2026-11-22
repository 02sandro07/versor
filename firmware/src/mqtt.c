/*
 * Copyright (c) 2026 Cadenced
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * One-shot TLS/MQTT publish for bring-up: prove the full data path
 * R1A -> LTE-M -> HiveMQ -> NAS -> website by pushing a single message.
 *
 * This mirrors what the DK / production firmware do, but as a blocking,
 * connect-publish-disconnect sequence driven from the `bu pub` shell command
 * (no zbus/state machine). Broker, credentials, topic and payload match the
 * device ID / serial number P0.1.1 (prototype R1A, unit 1) as used by the dashboard.
 *
 *   bu pub            -> publishes a default JSON telemetry sample
 *   bu pub <text...>  -> publishes <text...> verbatim (test any payload live)
 *
 * TLS uses the CA on BROKER_SEC_TAG, provisioned by credentials.c at boot.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/random/random.h>
#include <string.h>

#include "bringup.h"
#if defined(CONFIG_DATE_TIME)
#include <date_time.h>
#endif

LOG_MODULE_REGISTER(mqtt, LOG_LEVEL_INF);

/* Same broker as cell.c / the production firmware. */
#define BROKER_HOST "9d3a3c61e89e4384ab8e326039d9f567.s1.eu.hivemq.cloud"
#define BROKER_PORT 8883

/* Topics are built from the device ID (serial number, e.g. P0.1.1 — see
 * devcfg.c; same ID in the dashboard and in the Cockpit inventory):
 *   versor/data/<id>  uplink: telemetry + status/FOTA lines
 *   versor/evt/<id>   uplink: trip reports (start/stop/report)
 *   versor/cmd/<id>   downlink commands, e.g. {"cmd":"trip","a":"start"}
 *   versor/cfg/<id>   downlink config, RETAINED by the backend, e.g.
 *                      {"mode":"normal","int":300,"gnss":300,"led":0} — the
 *                      broker hands it over on every subscribe, so a device in
 *                      "normal" (PSM) mode picks up changes at its next cycle.
 */
#define CMD_SUB_ID  1
static char pub_topic[48];
static char raw_topic[48];
static char cmd_topic[48];
static char cfg_topic[48];

/* HiveMQ credentials — kept out of git in app_credentials.h (see .example).
 * Falls back to placeholders so the tree still builds without the secret file;
 * the broker will reject those, so provide the real app_credentials.h to publish.
 */
#if __has_include("app_credentials.h")
#include "app_credentials.h"
#endif
#ifndef APP_MQTT_USERNAME
#define APP_MQTT_USERNAME "innocowuser"
#endif
#ifndef APP_MQTT_PASSWORD
#define APP_MQTT_PASSWORD "CHANGME"
#endif
#define MQTT_USER   APP_MQTT_USERNAME
#define MQTT_PASS   APP_MQTT_PASSWORD

#define RX_BUF_SIZE 512
#define TX_BUF_SIZE 512
/* Telemetry JSON (see telemetry.c) is ~700 bytes; the payload is written
 * straight from this buffer (only the MQTT header uses tx_buffer).
 */
#define PAYLOAD_MAX 1024

static uint8_t rx_buffer[RX_BUF_SIZE];
static uint8_t tx_buffer[TX_BUF_SIZE];
static char client_id[48];

static struct mqtt_client client;
static struct sockaddr_storage broker;

static sec_tag_t sec_tag_list[] = { BROKER_SEC_TAG };

/* Event bookkeeping for the blocking loop. */
static volatile bool connected;
static volatile bool disconnected;
static volatile bool publish_acked;
static volatile int  connack_result = -1;

/* Stage-by-stage publish diagnostic, read via JLink (address in zephyr.map). */
struct pub_diag {
	volatile uint32_t magic;   /* 0x50FB0001 once run           */
	volatile int32_t  stage;   /* how far we got                */
	volatile int32_t  provca;  /* bringup_provision_ca()        */
	volatile int32_t  attach;  /* bringup_cell_attach()         */
	volatile int32_t  resolve; /* broker_resolve() (DNS)        */
	volatile int32_t  connect; /* mqtt_connect()                */
	volatile int32_t  connack_wait; /* wait_for(connected)      */
	volatile int32_t  connack;      /* CONNACK return_code      */
	volatile int32_t  publish; /* mqtt_publish()                */
	volatile int32_t  puback;  /* wait_for(publish_acked)       */
};
volatile struct pub_diag g_pub;
#define PUBMAGIC 0x50FB0001

/* Dispatch a downlink command payload received on the cmd topic:
 *   {"cmd":"fota","url":"https://.../app_update.bin"}   start a FOTA download
 *   {"cmd":"cfg","mode":"drain","int":10,...}          same as the cfg topic
 *   {"cmd":"led","on":1,"c":6}                         LED c: colour 0 off,
 *                                                      1 green, 2 blue, 3 red,
 *                                                      +4 = blink; without "c":
 *                                                      on = green
 *   {"cmd":"id","id":"P0.1.2"}                         set serial + reboot
 *   {"cmd":"reboot"}
 *   {"cmd":"trip","a":"start|stop|get"}                shipment trip; a report
 *                                                      follows on versor/evt/<id>
 */
static void handle_command(const char *payload)
{
	char cmd[16];

	if (!json_get_str(payload, "cmd", cmd, sizeof(cmd))) {
		LOG_WRN("Downlink without \"cmd\" field: %s", payload);
		return;
	}

	if (strcmp(cmd, "fota") == 0) {
		char url[256];

		if (!json_get_str(payload, "url", url, sizeof(url))) {
			LOG_ERR("fota command is missing the \"url\" field");
			return;
		}
		LOG_INF("FOTA command received -> %s", url);
		IF_ENABLED(CONFIG_FOTA_DOWNLOAD, ((void)bringup_fota_start_url(url);));
	} else if (strcmp(cmd, "cfg") == 0) {
		(void)devcfg_apply_json(payload);
		(void)app_shipment_cfg(payload);
	} else if (strcmp(cmd, "led") == 0) {
		int32_t on, c;
		char j[16];

		if (json_get_int(payload, "c", &c)) {
			snprintk(j, sizeof(j), "{\"led\":%d}", (int)c);
			(void)devcfg_apply_json(j);
		} else if (json_get_int(payload, "on", &on)) {
			(void)devcfg_apply_json(on ? "{\"led\":1}" : "{\"led\":0}");
		}
	} else if (strcmp(cmd, "id") == 0) {
		char id[24];

		if (json_get_str(payload, "id", id, sizeof(id)) &&
		    devcfg_set_device_id(id) == 0) {
			app_request_reboot();
		} else {
			LOG_ERR("id command: missing/invalid \"id\"");
		}
	} else if (strcmp(cmd, "reboot") == 0) {
		app_request_reboot();
	} else if (strcmp(cmd, "trip") == 0) {
		char a[8] = "get";

		(void)json_get_str(payload, "a", a, sizeof(a));
		app_trip_cmd(!strcmp(a, "start") ? 1 : !strcmp(a, "stop") ? 0 : 2);
	} else {
		LOG_WRN("Unknown command \"%s\" (ignored)", cmd);
	}
}

static void mqtt_evt_handler(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		connack_result = evt->param.connack.return_code;
		if (evt->result == 0 && connack_result == MQTT_CONNECTION_ACCEPTED) {
			connected = true;
			LOG_INF("CONNACK: connection accepted");
		} else {
			LOG_ERR("CONNACK: result=%d return_code=%d",
				evt->result, connack_result);
			disconnected = true;
		}
		break;
	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT disconnected (result %d)", evt->result);
		disconnected = true;
		break;
	case MQTT_EVT_PUBACK:
		LOG_INF("PUBACK id=%u result=%d",
			evt->param.puback.message_id, evt->result);
		publish_acked = true;
		break;
	case MQTT_EVT_SUBACK:
		LOG_INF("SUBACK id=%u — subscribed to %s + %s",
			evt->param.suback.message_id, cmd_topic, cfg_topic);
		break;
	case MQTT_EVT_PUBLISH: {
		/* A downlink arrived on the cmd or cfg topic. Read the payload,
		 * ack it (QoS1), and dispatch. Runs on the main thread (inside
		 * mqtt_input), so calling bringup_fota_start_url() — which is
		 * async — or writing settings from here is safe.
		 */
		const struct mqtt_publish_param *p = &evt->param.publish;
		static uint8_t cmd_buf[512];
		uint32_t len = p->message.payload.len;

		if (len >= sizeof(cmd_buf)) {
			len = sizeof(cmd_buf) - 1;
		}

		int r = mqtt_readall_publish_payload(c, cmd_buf, len);

		if (r == 0) {
			cmd_buf[len] = '\0';
			if (p->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE) {
				struct mqtt_puback_param ack = {
					.message_id = p->message_id,
				};
				(void)mqtt_publish_qos1_ack(c, &ack);
			}
			LOG_INF("Downlink on %.*s: %s",
				p->message.topic.topic.size,
				(const char *)p->message.topic.topic.utf8,
				cmd_buf);
			if (p->message.topic.topic.size == strlen(cfg_topic) &&
			    memcmp(p->message.topic.topic.utf8, cfg_topic,
				   strlen(cfg_topic)) == 0) {
				(void)devcfg_apply_json((const char *)cmd_buf);
				(void)app_shipment_cfg((const char *)cmd_buf);
			} else {
				handle_command((const char *)cmd_buf);
			}
		} else {
			LOG_ERR("mqtt_readall_publish_payload failed: %d", r);
		}
		break;
	}
	case MQTT_EVT_PINGRESP:
		break;
	default:
		LOG_DBG("MQTT event %d (result %d)", evt->type, evt->result);
		break;
	}
}

static int broker_resolve(void)
{
	struct addrinfo *res = NULL;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	int err;

	err = getaddrinfo(BROKER_HOST, NULL, &hints, &res);
	if (err) {
		LOG_ERR("getaddrinfo failed: %d (no data connectivity?)", err);
		return -EIO;
	}

	struct sockaddr_in *b = (struct sockaddr_in *)&broker;

	*b = *((struct sockaddr_in *)res->ai_addr);
	b->sin_family = AF_INET;
	b->sin_port = htons(BROKER_PORT);

	freeaddrinfo(res);
	return 0;
}

static bool client_inited;

static void client_init(void)
{
	mqtt_client_init(&client);
	client_inited = true;

	snprintk(client_id, sizeof(client_id), "%s-vsr-%08x",
		 devcfg_device_id(), sys_rand32_get());

	client.broker = &broker;
	client.evt_cb = mqtt_evt_handler;
	client.client_id.utf8 = client_id;
	client.client_id.size = strlen(client_id);
	client.protocol_version = MQTT_VERSION_3_1_1;

	static struct mqtt_utf8 user = { .utf8 = MQTT_USER,
					 .size = sizeof(MQTT_USER) - 1 };
	static struct mqtt_utf8 pass = { .utf8 = MQTT_PASS,
					 .size = sizeof(MQTT_PASS) - 1 };
	client.user_name = &user;
	client.password = &pass;

	client.rx_buf = rx_buffer;
	client.rx_buf_size = sizeof(rx_buffer);
	client.tx_buf = tx_buffer;
	client.tx_buf_size = sizeof(tx_buffer);

	/* TLS transport, offloaded to the modem via the provisioned sec tag. */
	client.transport.type = MQTT_TRANSPORT_SECURE;

	struct mqtt_sec_config *tls = &client.transport.tls.config;

	tls->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	tls->cipher_count = 0;
	tls->cipher_list = NULL;
	tls->sec_tag_count = ARRAY_SIZE(sec_tag_list);
	tls->sec_tag_list = sec_tag_list;
	tls->hostname = BROKER_HOST;
	tls->set_native_tls = false;
	/* Resume the TLS session on reconnect: in "normal" mode the device
	 * connects every cycle, and a resumed handshake is much shorter.
	 */
	tls->session_cache = TLS_SESSION_CACHE_ENABLED;
}

/* Pump the socket until `flag` becomes true or the timeout elapses. */
static int wait_for(volatile bool *flag, int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	struct zsock_pollfd fds = {
		.fd = client.transport.tls.sock,
		.events = ZSOCK_POLLIN,
	};

	while (!*flag && !disconnected) {
		int remaining = (int)(deadline - k_uptime_get());

		if (remaining <= 0) {
			return -ETIMEDOUT;
		}

		/* Keep the poll short so mqtt_live() runs for keepalive. */
		int poll_to = MIN(remaining, 1000);
		int ret = zsock_poll(&fds, 1, poll_to);

		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
			return -EIO;
		}
		if (ret > 0 && (fds.revents & ZSOCK_POLLIN)) {
			ret = mqtt_input(&client);
			if (ret) {
				LOG_ERR("mqtt_input error: %d", ret);
				return ret;
			}
		}
		(void)mqtt_live(&client);
		app_watchdog_kick();
	}

	return disconnected && !*flag ? -ENOTCONN : 0;
}

/* Session state: true between a successful session_start() and session_stop(). */
static bool session_up;

bool bringup_mqtt_session_is_up(void)
{
	return session_up;
}

/* Publish one QoS1 message on the (already connected) session client and pump
 * briefly for its PUBACK. Session must be up.
 */
static int session_publish_to(const char *topic, const char *payload, bool log);

int bringup_mqtt_session_publish(const char *payload)
{
	return session_publish_to(pub_topic, payload, true);
}

int bringup_mqtt_session_publish_raw(const char *payload)
{
	return session_publish_to(raw_topic, payload, false);
}

static int session_publish_to(const char *topic, const char *payload, bool log)
{
	char default_payload[PAYLOAD_MAX];
	int err;

	if (!session_up) {
		LOG_ERR("MQTT session not up — call bringup_mqtt_session_start first");
		return -ENOTCONN;
	}

	if (payload == NULL) {
		if (telemetry_format_latest(default_payload, sizeof(default_payload)) < 0) {
			return -ENOMEM;
		}
		payload = default_payload;
	}

	publish_acked = false;

	uint16_t msg_id = (uint16_t)(sys_rand32_get() | 1U);
	struct mqtt_publish_param param = {
		.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE,
		.message.topic.topic.utf8 = topic,
		.message.topic.topic.size = strlen(topic),
		.message.payload.data = (uint8_t *)payload,
		.message.payload.len = strlen(payload),
		.message_id = msg_id,
		.dup_flag = 0,
		.retain_flag = 0,
	};

	err = mqtt_publish(&client, &param);
	if (err) {
		LOG_ERR("mqtt_publish failed: %d — treating session as down", err);
		session_up = false;
		return -ENOTCONN;
	}
	if (log) {
		LOG_INF("Published to \"%s\": %s", topic, payload);
	} else {
		LOG_INF("Published %u B to \"%s\"", (unsigned)strlen(payload), topic);
	}

	err = wait_for(&publish_acked, 10000);
	if (err == -ENOTCONN) {
		session_up = false;
		LOG_WRN("Session dropped while waiting for PUBACK");
		return err;
	}
	if (err) {
		LOG_WRN("No PUBACK (%d) — message may still have been delivered", err);
	} else {
		LOG_INF("Publish confirmed (PUBACK)");
	}
	return err;
}

/* Provision CA (offline) + attach LTE + resolve + TLS connect + wait CONNACK. */
int bringup_mqtt_session_start(void)
{
	int err;

	if (session_up) {
		return 0;
	}
	/* A previous session that dropped may still hold its socket. */
	if (client_inited) {
		mqtt_abort(&client);
	}

	err = bringup_provision_ca();
	if (err) {
		LOG_ERR("CA not provisioned — cannot set up TLS");
		return -EIO;
	}

	err = bringup_cell_attach();
	if (err) {
		LOG_ERR("LTE not attached — cannot connect MQTT");
		return -ENETDOWN;
	}

	err = broker_resolve();
	if (err) {
		return err;
	}

	snprintk(pub_topic, sizeof(pub_topic), "versor/data/%s", devcfg_device_id());
	snprintk(raw_topic, sizeof(raw_topic), "versor/evt/%s", devcfg_device_id());
	snprintk(cmd_topic, sizeof(cmd_topic), "versor/cmd/%s", devcfg_device_id());
	snprintk(cfg_topic, sizeof(cfg_topic), "versor/cfg/%s", devcfg_device_id());

	client_init();

	connected = false;
	disconnected = false;
	publish_acked = false;
	connack_result = -1;

	LOG_INF("Connecting (TLS) to %s:%d as %s ...",
		BROKER_HOST, BROKER_PORT, client_id);

	err = mqtt_connect(&client);
	if (err) {
		LOG_ERR("mqtt_connect failed: %d (CA on sec tag %d? credentials?)",
			err, BROKER_SEC_TAG);
		return err;
	}

	err = wait_for(&connected, 15000);
	if (err) {
		LOG_ERR("No CONNACK: %d (return_code=%d)", err, connack_result);
		(void)mqtt_disconnect(&client, NULL);
		return err;
	}

	session_up = true;

	/* Subscribe to the downlink command + (retained) config topics. Best
	 * effort: if this fails, telemetry publishing still works — only remote
	 * commands are unavailable until the next reconnect re-subscribes.
	 */
	struct mqtt_topic sub_topics[] = {
		{
			.topic.utf8 = cmd_topic,
			.topic.size = strlen(cmd_topic),
			.qos = MQTT_QOS_1_AT_LEAST_ONCE,
		},
		{
			.topic.utf8 = cfg_topic,
			.topic.size = strlen(cfg_topic),
			.qos = MQTT_QOS_1_AT_LEAST_ONCE,
		},
	};
	struct mqtt_subscription_list sub_list = {
		.list = sub_topics,
		.list_count = ARRAY_SIZE(sub_topics),
		.message_id = CMD_SUB_ID,
	};

	err = mqtt_subscribe(&client, &sub_list);
	if (err) {
		LOG_WRN("mqtt_subscribe failed: %d — downlink disabled", err);
	} else {
		LOG_INF("Subscribing to %s + %s", cmd_topic, cfg_topic);
	}

	LOG_INF("MQTT session up");
	return 0;
}

/* Pump the socket for up to timeout_ms so keepalive pings go out and any
 * broker traffic is serviced while the app is otherwise idle between publishes.
 */
int bringup_mqtt_session_poll(int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	if (!session_up) {
		return -ENOTCONN;
	}

	struct zsock_pollfd fds = {
		.fd = client.transport.tls.sock,
		.events = ZSOCK_POLLIN,
	};

	while (k_uptime_get() < deadline && !disconnected) {
		int remaining = (int)(deadline - k_uptime_get());
		int poll_to = MIN(remaining, 1000);
		int ret = zsock_poll(&fds, 1, poll_to);

		if (ret > 0 && (fds.revents & ZSOCK_POLLIN)) {
			if (mqtt_input(&client) != 0) {
				break;
			}
		}
		(void)mqtt_live(&client);
	}

	if (disconnected) {
		session_up = false;
		LOG_WRN("MQTT session dropped");
		return -ENOTCONN;
	}
	return 0;
}

void bringup_mqtt_session_stop(void)
{
	if (!session_up) {
		return;
	}
	(void)mqtt_disconnect(&client, NULL);
	for (int i = 0; i < 5 && !disconnected; i++) {
		if (mqtt_input(&client) != 0) {
			break;
		}
		k_msleep(50);
	}
	session_up = false;
	LOG_INF("MQTT session stopped");
}

/* One-shot publish for the `bu pub` shell command: connect, publish, close. */
int bringup_mqtt_pub(const char *payload)
{
	int err;

	g_pub.magic = PUBMAGIC;
	g_pub.stage = 1;

	err = bringup_mqtt_session_start();
	g_pub.attach = err;
	if (err) {
		return err;
	}
	g_pub.stage = 5;

	err = bringup_mqtt_session_publish(payload);
	g_pub.publish = err;
	g_pub.stage = err ? 7 : 8;

	bringup_mqtt_session_stop();
	return err;
}
