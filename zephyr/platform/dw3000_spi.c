/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <device.h>
#include <drivers/spi.h>
#include <logging/log.h>
#include <stdio.h>
#include <zephyr.h>

#include "dw3000_spi.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_DECLARE(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define TX_WAIT_RESP_NRF52840_DELAY 30

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, decawave_dw3000))

static const struct device* spi;
static struct spi_cs_control* cs_ctrl = SPI_CS_CONTROL_PTR_DT(DW_INST, 0);
static struct spi_config spi_cfgs[2] = {0}; // configs for slow and fast
static struct spi_config* spi_cfg;

#define DEBUG_SPI_TRACE 1

#if DEBUG_SPI_TRACE
#define DBGS_CNT  256
#define DBGS_BODY 70

struct spi_dbg {
	bool rw;
	uint8_t hdr[2];
	uint8_t hdr_len;
	uint8_t bdy[DBGS_BODY];
	uint8_t bdy_len;
};

static struct spi_dbg dbgs[DBGS_CNT];
static int dbgs_cnt;
#endif

int dw3000_spi_init(void)
{
	/* set common SPI config */
	for (int i = 0; i < ARRAY_SIZE(spi_cfgs); i++) {
		spi_cfgs[i].cs = cs_ctrl;
		spi_cfgs[i].operation = SPI_WORD_SET(8);
	}

	/* Slow SPI clock speed: 2MHz */
	spi_cfgs[0].frequency = 2000000;

	/* High SPI clock speed: assume 8MHz for all boards */
	spi_cfgs[1].frequency = 8000000;

	/* High SPI clock speed: increase for boards which support higher speeds */
#if CONFIG_SOC_NRF52833 || CONFIG_SOC_NRF52840
#if CONFIG_SHIELD_QORVO_DWS3000
	/* Due to the wiring of the Nordic Development Boards and the DWS3000
	 * Arduino shield it is not possible to use more than 16MHz */
	spi_cfgs[1].frequency = 16000000;
#else
	spi_cfgs[1].frequency = 32000000;
#endif
#endif

	spi_cfg = &spi_cfgs[0];

	spi = device_get_binding(DT_LABEL(DW_SPI));
	if (!spi) {
		LOG_ERR("DW3000 SPI binding failed");
		return -1;
	} else {
		LOG_INF("DW3000 on %s (max %dMHz)", DT_LABEL(DW_SPI),
				spi_cfgs[1].frequency / 1000000);
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

static char* spi_dbg_out_reg(bool rw, const uint8_t* headerBuffer,
							 uint16_t headerLength)
{
	static char buf[64];
	uint8_t reg = 0;
	uint8_t mode = 0;
	uint8_t sub = 0;

	reg = (headerBuffer[0] & 0x3e) >> 1;
	mode = headerBuffer[0] & 0xc1;

	if (headerLength > 1) {
		sub = (headerBuffer[1] & 0xFC) >> 2;
		sub |= (headerBuffer[0] & 0x01) << 6;
	}

	char* modestr = "-unk-";
	if (mode == 0x81)
		modestr = "fast";
	else if (mode == 0x80)
		modestr = "short write";
	else if (mode == 0x00)
		modestr = "short read";
	else if ((mode & 0xc0) == 0x40)
		modestr = "full read";
	else if ((mode & 0xc0) == 0xc0) {
		uint8_t mask = headerBuffer[1] & 0x03;
		if (mask == 0x00)
			modestr = "full write";
		else if (mask == 0x01)
			modestr = "masked 8bit";
		else if (mask == 0x02)
			modestr = "masked 16bit";
		else if (mask == 0x03)
			modestr = "masked 32bit";
	}

	snprintf(buf, sizeof(buf), "SPI %s %02X:%02X (%s): ", rw ? "READ" : "WRITE",
			 reg, sub, modestr);
	return buf;
}

static void hexdump(const char* txt, const uint8_t* data, int len)
{
	char buf[4];
	LOG_PRINTK("%s", txt);

	for (size_t i = 0; i < len; i++) {
		int res = snprintf(buf, sizeof(buf), "%02x ", data[i]);
		if (res > 0) {
			LOG_PRINTK("%s", buf);
		}
	}
	LOG_PRINTK("\n");
}

static void spi_dbg_in(bool rw, const uint8_t* headerBuffer,
					   uint16_t headerLength, const uint8_t* bodyBuffer,
					   uint16_t bodyLength)
{
#if DEBUG_SPI_TRACE
#if 0
    dbgout ("---SPI #%d %s", dbgs_cnt, rw ? "READ" : "WRITE");
    hexdump("   SPI HEADER: ", headerBuffer, headerLength);
    hexdump("   SPI BODY: ", bodyBuffer, bodyLength);
#endif

	if (dbgs_cnt >= DBGS_CNT) {
		LOG_ERR("ERR not enough debug space");
		return;
	}

	dbgs[dbgs_cnt].rw = rw;
	if (headerLength <= 2)
		memcpy(dbgs[dbgs_cnt].hdr, headerBuffer, headerLength);
	else
		LOG_ERR("ERR hdr len %d", headerLength);
	dbgs[dbgs_cnt].hdr_len = headerLength;
	if (bodyLength <= DBGS_BODY)
		memcpy(dbgs[dbgs_cnt].bdy, bodyBuffer, bodyLength);
	else
		LOG_ERR("ERR bdy len %d at #%d", bodyLength, dbgs_cnt);
	dbgs[dbgs_cnt].bdy_len = bodyLength;

#if 0
	// Real-time SPI debug output (normally too slow)
	char* s = spi_dbg_out_reg(dbgs[dbgs_cnt].rw, dbgs[dbgs_cnt].hdr, dbgs[dbgs_cnt].hdr_len);
	hexdump(s, dbgs[dbgs_cnt].bdy, dbgs[dbgs_cnt].bdy_len);
#endif

	dbgs_cnt++;
#endif
}

void dw3000_spi_dbg_output()
{
#if DEBUG_SPI_TRACE
	LOG_PRINTK("--- SPI DBG START\n");
	for (int i = 0; i < DBGS_CNT && i < dbgs_cnt; i++) {
		struct spi_dbg* d = &dbgs[i];
		// hexdump("   SPI HEADER: ", d->hdr, d->hdr_len);
		char* s = spi_dbg_out_reg(d->rw, d->hdr, d->hdr_len);
		hexdump(s, d->bdy, d->bdy_len);
	}
	dbgs_cnt = 0;
	LOG_PRINTK("--- SPI DBG END\n");
#endif
}

int dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
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

int dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
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

	spi_dbg_in(false, headerBuffer, headerLength, bodyBuffer, bodyLength);

	return spi_transceive(spi, spi_cfg, &tx, NULL);
}

int dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
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

	spi_dbg_in(true, headerBuffer, headerLength, readBuffer, readLength);

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
	gpio_pin_set_dt(&cs_ctrl->gpio, 1);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl->gpio, 0);
}
