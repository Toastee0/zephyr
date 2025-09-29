/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_nrf54l15_hpf

#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/ipc_service.h>
#include <zephyr/ipc/ipc_service.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_nrf54l15_hpf);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/sys/util_macro.h>

/* Include HPF data structures from Nordic SDK */
#include <drivers/led_strip/hpf_ws2812.h>

struct ws2812_nrf54l15_hpf_cfg {
	uint32_t pin;
	uint8_t port;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	size_t length;
	/* Communication backend configuration */
	const struct device *mbox_dev;
	uint32_t mbox_channel;
	bool use_icmsg; /* true for ICMSG, false for MBOX */
	const char *ipc_ept_name;
};

struct ws2812_nrf54l15_hpf_data {
	struct k_sem tx_sem;
	/* ICMSG backend data */
	struct ipc_ept ept;
	/* MBOX backend data */
	struct mbox_msg mbox_msg;
	/* HPF communication data */
	hpf_ws2812_mbox_data_t hpf_data;
};

/* MBOX backend communication functions */
static int ws2812_hpf_mbox_send_packet(const struct device *dev, 
                                       hpf_ws2812_opcode_t opcode,
                                       uint32_t pin, uint8_t port, uint32_t numleds)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	int ret;

	if (!config->mbox_dev) {
		return -ENODEV;
	}

	/* Prepare HPF data packet */
	data->hpf_data.data.opcode = opcode;
	data->hpf_data.data.pin = pin;
	data->hpf_data.data.port = port;
	data->hpf_data.data.numleds = numleds;

	/* Prepare MBOX message */
	data->mbox_msg.data = &data->hpf_data;
	data->mbox_msg.size = sizeof(data->hpf_data);

	/* Send via MBOX */
	ret = mbox_send(config->mbox_dev, config->mbox_channel, &data->mbox_msg);
	if (ret < 0) {
		LOG_ERR("MBOX send failed: %d", ret);
		return ret;
	}

	return 0;
}

/* ICMSG backend communication functions */
static void icmsg_ept_bound(void *priv)
{
	struct ws2812_nrf54l15_hpf_data *data = (struct ws2812_nrf54l15_hpf_data *)priv;
	
	k_sem_give(&data->tx_sem);
}

static void icmsg_ept_recv(const void *msg_data, size_t len, void *priv)
{
	/* Handle response from FLPR core if needed */
	LOG_DBG("Received response from FLPR, len: %zu", len);
}

static int ws2812_hpf_icmsg_send_packet(const struct device *dev,
                                        hpf_ws2812_opcode_t opcode,
                                        uint32_t pin, uint8_t port, uint32_t numleds)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	hpf_ws2812_data_packet_t packet;
	int ret;

	/* Prepare HPF data packet */
	packet.opcode = opcode;
	packet.pin = pin;
	packet.port = port;
	packet.numleds = numleds;

	/* Send via ICMSG */
	ret = ipc_service_send(&data->ept, &packet, sizeof(packet));
	if (ret < 0) {
		LOG_ERR("ICMSG send failed: %d", ret);
		return ret;
	}

	return 0;
}

static int send_hpf_command(const struct device *dev,
                           hpf_ws2812_opcode_t opcode,
                           uint32_t pin, uint8_t port, uint32_t numleds)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;

	if (config->use_icmsg) {
		return ws2812_hpf_icmsg_send_packet(dev, opcode, pin, port, numleds);
	} else {
		return ws2812_hpf_mbox_send_packet(dev, opcode, pin, port, numleds);
	}
}

static int ws2812_nrf54l15_hpf_update_rgb(const struct device *dev,
					  struct led_rgb *pixels,
					  size_t num_pixels)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	uint8_t *pixel_data = (uint8_t *)pixels;
	size_t i;
	int ret;

	if (num_pixels > config->length) {
		LOG_ERR("Too many pixels: %zu (max: %zu)", num_pixels, config->length);
		return -EINVAL;
	}

	/* Take semaphore to ensure exclusive access */
	ret = k_sem_take(&data->tx_sem, K_MSEC(100));
	if (ret != 0) {
		LOG_ERR("Failed to acquire tx semaphore: %d", ret);
		return ret;
	}

	/* Convert from RGB to on-wire format (e.g. GRB, GRBW, RGB, etc) */
	for (i = 0; i < num_pixels; i++) {
		uint8_t j;
		const struct led_rgb current_pixel = pixels[i];

		for (j = 0; j < config->num_colors; j++) {
			switch (config->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				*pixel_data++ = 0;
				break;
			case LED_COLOR_ID_RED:
				*pixel_data++ = current_pixel.r;
				break;
			case LED_COLOR_ID_GREEN:
				*pixel_data++ = current_pixel.g;
				break;
			case LED_COLOR_ID_BLUE:
				*pixel_data++ = current_pixel.b;
				break;
			default:
				k_sem_give(&data->tx_sem);
				return -EINVAL;
			}
		}
	}

	/* TODO: Send pixel data to FLPR core via HPF
	 * This will require extending the HPF protocol to handle
	 * pixel data transfer, possibly via shared memory.
	 * For now, send the refresh command.
	 */
	ret = send_hpf_command(dev, HPF_WS2812_REFRESH, 0, 0, num_pixels);

	k_sem_give(&data->tx_sem);

	return ret;
}

static size_t ws2812_nrf54l15_hpf_length(const struct device *dev)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;

	return config->length;
}

static const DEVICE_API(led_strip, ws2812_nrf54l15_hpf_api) = {
	.update_rgb = ws2812_nrf54l15_hpf_update_rgb,
	.length = ws2812_nrf54l15_hpf_length,
};

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)					\
static const uint8_t ws2812_nrf54l15_hpf_##idx##_color_mapping[] =	\
	DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

#define WS2812_NRF54L15_HPF_DEVICE(idx)				\
									\
	static int ws2812_nrf54l15_hpf_##idx##_init(const struct device *dev) \
	{								\
		const struct ws2812_nrf54l15_hpf_cfg *cfg = dev->config; \
		struct ws2812_nrf54l15_hpf_data *data = dev->data;	\
		uint8_t i;						\
		int ret;						\
									\
		/* Initialize semaphore */					\
		k_sem_init(&data->tx_sem, 1, 1);			\
									\
		/* Validate color mapping */				\
		for (i = 0; i < cfg->num_colors; i++) {			\
			switch (cfg->color_mapping[i]) {		\
			case LED_COLOR_ID_WHITE:			\
			case LED_COLOR_ID_RED:				\
			case LED_COLOR_ID_GREEN:			\
			case LED_COLOR_ID_BLUE:				\
				break;					\
			default:					\
				LOG_ERR("%s: invalid channel to color mapping." \
					" Check the color-mapping DT property",	\
					dev->name);			\
				return -EINVAL;				\
			}						\
		}							\
									\
		/* Initialize communication backend */			\
		if (cfg->use_icmsg) {					\
			/* Initialize ICMSG endpoint */			\
			struct ipc_ept_cfg ept_cfg = {			\
				.name = cfg->ipc_ept_name,		\
				.cb = {					\
					.bound = icmsg_ept_bound,	\
					.received = icmsg_ept_recv,	\
				},					\
				.priv = data,				\
			};						\
									\
			ret = ipc_service_open_instance(cfg->mbox_dev);	\
			if (ret < 0 && ret != -EALREADY) {		\
				LOG_ERR("IPC service open failed: %d", ret); \
				return ret;				\
			}						\
									\
			ret = ipc_service_register_endpoint(cfg->mbox_dev, \
							     &data->ept, &ept_cfg); \
			if (ret < 0) {					\
				LOG_ERR("IPC endpoint register failed: %d", ret); \
				return ret;				\
			}						\
									\
			/* Wait for endpoint to be bound */		\
			ret = k_sem_take(&data->tx_sem, K_SECONDS(1));	\
			if (ret != 0) {					\
				LOG_ERR("ICMSG endpoint binding timeout");	\
				return -ETIMEDOUT;			\
			}						\
		} else {						\
			/* MBOX backend - device should be ready */	\
			if (!device_is_ready(cfg->mbox_dev)) {		\
				LOG_ERR("MBOX device not ready");	\
				return -ENODEV;				\
			}						\
		}							\
									\
		/* Configure GPIO pin on FLPR core */			\
		ret = send_hpf_command(dev, HPF_WS2812_PIN_CONFIGURE,	\
				       cfg->pin, cfg->port, cfg->length); \
		if (ret < 0) {						\
			LOG_ERR("HPF pin configure failed: %d", ret);	\
			return ret;					\
		}							\
									\
		LOG_INF("WS2812 nRF54L15 HPF driver initialized: "	\
			"pin P%d.%02d, %zu LEDs, %s backend",		\
			cfg->port, cfg->pin, cfg->length,		\
			cfg->use_icmsg ? "ICMSG" : "MBOX");		\
									\
		return 0;						\
	}								\
									\
	BUILD_ASSERT(WS2812_NUM_COLORS(idx) <= sizeof(struct led_rgb),	\
		"Too many channels in color-mapping; "			\
		"currently not supported by the ws2812_nrf54l15_hpf driver"); \
									\
	WS2812_COLOR_MAPPING(idx);					\
									\
	static const struct ws2812_nrf54l15_hpf_cfg ws2812_nrf54l15_hpf_##idx##_cfg = { \
		.pin = DT_INST_PROP(idx, pin),				\
		.port = DT_INST_PROP(idx, port),			\
		.num_colors = WS2812_NUM_COLORS(idx),			\
		.color_mapping = ws2812_nrf54l15_hpf_##idx##_color_mapping, \
		.length = DT_INST_PROP(idx, chain_length),		\
		.mbox_dev = DEVICE_DT_GET(DT_INST_PHANDLE(idx, mbox_dev)), \
		.mbox_channel = DT_INST_PROP(idx, mbox_channel),	\
		.use_icmsg = DT_INST_PROP_OR(idx, use_icmsg, false),	\
		.ipc_ept_name = DT_INST_PROP_OR(idx, ipc_ept_name, "ws2812_hpf"), \
	};								\
									\
	static struct ws2812_nrf54l15_hpf_data ws2812_nrf54l15_hpf_##idx##_data; \
									\
	DEVICE_DT_INST_DEFINE(idx,					\
			    ws2812_nrf54l15_hpf_##idx##_init,		\
			    NULL,					\
			    &ws2812_nrf54l15_hpf_##idx##_data,		\
			    &ws2812_nrf54l15_hpf_##idx##_cfg, POST_KERNEL, \
			    CONFIG_LED_STRIP_INIT_PRIORITY,		\
			    &ws2812_nrf54l15_hpf_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_NRF54L15_HPF_DEVICE)