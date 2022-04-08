#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <logging/log.h>
#include <zephyr.h>

#include "deca_device_api.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_REGISTER(dw3000_spi);

#define DW_INST DT_INST(0, decawave_dw3000)

static struct spi_dt_spec spi = SPI_DT_SPEC_GET(DW_INST, SPI_WORD_SET(8), 0);

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

	return spi_transceive_dt(&spi, &tx, NULL);
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

	return spi_transceive_dt(&spi, &tx, NULL);
}

int readfromspi(uint16_t headerLength, /*const*/ uint8_t* headerBuffer,
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
		.buffers = &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	return spi_transceive_dt(&spi, &tx, &rx);
}

/* our own functions */

void dw3000_spi_speed_low(void)
{
	spi.config.frequency = 2000000;
}

void dw3000_spi_speed_high(void)
{
	spi.config.frequency = 8000000;
}
