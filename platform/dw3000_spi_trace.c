#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "dw3000_spi.h"

#if CONFIG_DW3000_SPI_TRACE

LOG_MODULE_REGISTER(dw3000, CONFIG_DW3000_LOG_LEVEL);

// Real-time output (normally too slow)
#ifndef CONFIG_DW3000_SPI_TRACE_REALTIME
#define CONFIG_DW3000_SPI_TRACE_REALTIME 0
#endif

#ifndef CONFIG_DW3000_SPI_TRACE_CNT
#define CONFIG_DW3000_SPI_TRACE_CNT 256
#endif

#ifndef CONFIG_DW3000_SPI_TRACE_BODY_LEN
#define CONFIG_DW3000_SPI_TRACE_BODY_LEN 12
#endif

#define DW3000_SPI_TRACE_RAW 0

struct spi_dbg {
	bool rw;
	uint8_t hdr[2];
	uint8_t hdr_len;
	uint8_t bdy[CONFIG_DW3000_SPI_TRACE_BODY_LEN];
	uint8_t bdy_len;
};

static struct spi_dbg dbgs[CONFIG_DW3000_SPI_TRACE_CNT];
static int dbgs_cnt;

static char* spi_dbg_out_reg(const char* prefix, bool rw,
							 const uint8_t* headerBuffer, uint16_t headerLength)
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
	if (mode == 0x81) {
		modestr = "fast";
	} else if (mode == 0x80) {
		modestr = "short write";
	} else if (mode == 0x00) {
		modestr = "short read";
	} else if ((mode & 0xc0) == 0x40) {
		modestr = "full read";
	} else if ((mode & 0xc0) == 0xc0) {
		uint8_t mask = headerBuffer[1] & 0x03;
		if (mask == 0x00) {
			modestr = "full write";
		} else if (mask == 0x01) {
			modestr = "masked 8bit";
		} else if (mask == 0x02) {
			modestr = "masked 16bit";
		} else if (mask == 0x03) {
			modestr = "masked 32bit";
		}
	}

	if (prefix) {
		snprintf(buf, sizeof(buf), "%s: SPI %s %02X:%02X (%s)", prefix,
				 rw ? "READ" : "WRITE", reg, sub, modestr);
	} else {
		snprintf(buf, sizeof(buf), "SPI %s %02X:%02X (%s)",
				 rw ? "READ" : "WRITE", reg, sub, modestr);
	}
	return buf;
}

void dw3000_spi_trace_in(bool rw, const uint8_t* headerBuffer,
						 uint16_t headerLength, const uint8_t* bodyBuffer,
						 uint16_t bodyLength)
{
#if DW3000_SPI_TRACE_RAW
	LOG_INF("---SPI #%d %s hdrlen %d bodylen %d", dbgs_cnt,
			rw ? "READ" : "WRITE", headerLength, bodyLength);
	LOG_HEXDUMP("   SPI HEADER: ", headerBuffer, headerLength);
	LOG_HEXDUMP("   SPI BODY: ", bodyBuffer, bodyLength);
#endif

	if (dbgs_cnt >= CONFIG_DW3000_SPI_TRACE_CNT) {
		LOG_ERR("ERR not enough debug space");
		return;
	}

	dbgs[dbgs_cnt].rw = rw;

	if (headerLength <= 2) {
		memcpy(dbgs[dbgs_cnt].hdr, headerBuffer, headerLength);
	} else {
		LOG_ERR("ERR hdr len %d", headerLength);
	}

	dbgs[dbgs_cnt].hdr_len = headerLength;
	if (bodyLength > CONFIG_DW3000_SPI_TRACE_BODY_LEN) {
		// LOG_ERR("ERR bdy len %d at #%d", bodyLength, dbgs_cnt);
		bodyLength = CONFIG_DW3000_SPI_TRACE_BODY_LEN;
	}
	memcpy(dbgs[dbgs_cnt].bdy, bodyBuffer, bodyLength);
	dbgs[dbgs_cnt].bdy_len = bodyLength;

#if CONFIG_DW3000_SPI_TRACE_REALTIME
	if (dbgs[dbgs_cnt].bdy_len) {
		char* s = spi_dbg_out_reg(LOG_TAG, dbgs[dbgs_cnt].rw,
								  dbgs[dbgs_cnt].hdr, dbgs[dbgs_cnt].hdr_len);
		LOG_HEXDUMP(s, dbgs[dbgs_cnt].bdy, dbgs[dbgs_cnt].bdy_len);
	} else {
		char* s = spi_dbg_out_reg(NULL, dbgs[dbgs_cnt].rw, dbgs[dbgs_cnt].hdr,
								  dbgs[dbgs_cnt].hdr_len);
		LOG_INF("%s", s);
	}
#endif

	dbgs_cnt++;
}

#endif

void dw3000_spi_trace_output(void)
{
#if CONFIG_DW3000_SPI_TRACE
	LOG_INF("--- SPI DBG START");
	for (int i = 0; i < CONFIG_DW3000_SPI_TRACE_CNT && i < dbgs_cnt; i++) {
		struct spi_dbg* d = &dbgs[i];
		if (d->bdy_len) {
			char* s = spi_dbg_out_reg(LOG_TAG, d->rw, d->hdr, d->hdr_len);
			LOG_HEXDUMP(s, d->bdy, d->bdy_len);
		} else {
			char* s = spi_dbg_out_reg(NULL, d->rw, d->hdr, d->hdr_len);
			LOG_INF("%s", s);
		}
	}
	dbgs_cnt = 0;
	LOG_INF("--- SPI DBG END");
#endif
}
