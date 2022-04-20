/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <device.h>
#include <drivers/spi.h>
#include <logging/log.h>
#include <zephyr.h>

#include "deca_device_api.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_REGISTER(dw3000_spi);

#define TX_WAIT_RESP_NRF52840_DELAY 30

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, decawave_dw3000))

static const struct device* spi;
static struct spi_cs_control* cs_ctrl = SPI_CS_CONTROL_PTR_DT(DW_INST, 0);
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;

int dw3000_spi_init(void)
{
	/* set common SPI config */
	for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++) {
		spi_cfgs[i].cs = cs_ctrl;
		spi_cfgs[i].operation = SPI_WORD_SET(8);
	}

	spi_cfgs[0].frequency = 2000000; // slow
	spi_cfgs[1].frequency = 8000000; // fast

	spi_cfg = &spi_cfgs[0];

	spi = device_get_binding(DT_LABEL(DW_SPI));
	if (!spi) {
		LOG_ERR("DW3000 SPI binding failed");
		return -1;
	} else {
		LOG_INF("DW3000 on %s", DT_LABEL(DW_SPI));
	}

	return 0;
}

void dw3000_spi_speed_slow(void)
{
	spi_cfg = &spi_cfgs[0];
}

void dw3000_spi_speed_fast(void)
{
	spi_cfg = &spi_cfgs[1];
}

void dw3000_spi_fini(void)
{
	// TODO
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t* headerBuffer,
					  uint16_t bodyLength, const uint8_t* bodyBuffer,
					  uint8_t crc8)
{
	const struct spi_buf tx_buf[3] = {
		{
			.buf = headerBuffer,
			.len = headerLength,
		},
		{
			.buf = bodyBuffer,
			.len = bodyLength,
		},
		{
			.buf = &crc8,
			.len = 1,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int writetospi(uint16_t headerLength, const uint8_t* headerBuffer,
			   uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	const struct spi_buf tx_buf[2] = {
		{
			.buf = headerBuffer,
			.len = headerLength,
		},
		{
			.buf = bodyBuffer,
			.len = bodyLength,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int readfromspi(uint16_t headerLength, uint8_t* headerBuffer,
				uint16_t readLength, uint8_t* readBuffer)
{
	const struct spi_buf tx_buf = {
		.buf = headerBuffer,
		.len = headerLength,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = headerLength,
		},
		{
			.buf = readBuffer,
			.len = readLength,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	int ret = spi_transceive(spi, spi_cfg, &tx, &rx);

#if (CONFIG_SOC_NRF52840_QIAA)
	/*
	 *  This is a hack to handle the corrupted response frame through the
	 * nRF52840's SPI3. See this project's issue-log
	 * (https://github.com/foldedtoad/dwm3000/issues/2) for details. The delay
	 * value is set in the CMakeList.txt file for this subproject.
	 */
	if (ret == 0) {
		for (volatile int i = 0; i < TX_WAIT_RESP_NRF52840_DELAY; i++) {
			/* spin */
		}
	}
#endif

	return ret;
}
