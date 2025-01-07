/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "dw3000_spi.h"

#include "version.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_DECLARE(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define TX_WAIT_RESP_NRF52840_DELAY 30

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, decawave_dw3000))

static const struct device* spi;
#if KERNEL_VERSION_MAJOR > 3                                                   \
	|| (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST, 0);
#else
static struct spi_cs_control* cs_ctrl = SPI_CS_CONTROL_PTR_DT(DW_INST, 0);
#endif
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;

int dw3000_spi_init(void)
{
	/* set common SPI config */
	for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++) {
		spi_cfgs[i].cs = cs_ctrl;
		spi_cfgs[i].operation = SPI_WORD_SET(8);
	}

	/* Slow SPI clock speed: 2MHz */
	spi_cfgs[0].frequency = 2000000;

	/* High SPI clock speed: increase for boards which support higher speeds */
#if (CONFIG_SOC_NRF52833 || CONFIG_SOC_NRF52840 || CONFIG_SOC_NRF5340_CPUAPP) \
    && CONFIG_SHIELD_QORVO_DWS3000 && (CONFIG_DW3000_SPI_MAX_MHZ > 16)
	/* Due to the wiring of the Nordic Development Boards and the DWS3000
	 * Arduino shield it is not possible to use more than 16MHz */
	spi_cfgs[1].frequency = 16000000;
#else
	spi_cfgs[1].frequency = CONFIG_DW3000_SPI_MAX_MHZ * 1000000;
#endif

	spi_cfg = &spi_cfgs[0];

	spi = DEVICE_DT_GET(DW_SPI);
	if (!spi) {
		LOG_ERR("DW3000 SPI binding failed");
		return -1;
	} else {
		LOG_INF("DW3000 (max %dMHz)", spi_cfgs[1].frequency / 1000000);
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

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
						 uint16_t bodyLength, const uint8_t* bodyBuffer,
						 uint8_t crc8)
{
	const struct spi_buf tx_buf[3] = {
		{
			.buf = (void*)headerBuffer,
			.len = headerLength,
		},
		{
			.buf = (void*)bodyBuffer,
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

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
					 uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	const struct spi_buf tx_buf[2] = {
		{
			.buf = (void*)headerBuffer,
			.len = headerLength,
		},
		{
			.buf = (void*)bodyBuffer,
			.len = bodyLength,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
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

void dw3000_spi_wakeup()
{
#if KERNEL_VERSION_MAJOR > 3                                                   \
	|| (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
#else
	gpio_pin_set_dt(&cs_ctrl->gpio, 0);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl->gpio, 1);
#endif
}
