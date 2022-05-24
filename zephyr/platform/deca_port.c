#include <kernel.h>

#include "deca_interface.h"
#include "dw3000.h"

/* This file implements the functions required by decadriver */

/* deca_spi.c */
int dw3000_writetospiwithcrc(uint16_t headerLength, const uint8_t* headerBuffer,
					  uint16_t bodyLength, const uint8_t* bodyBuffer,
					  uint8_t crc8);
int dw3000_writetospi(uint16_t headerLength, const uint8_t* headerBuffer,
			   uint16_t bodyLength, const uint8_t* bodyBuffer);
int dw3000_readfromspi(uint16_t headerLength, uint8_t* headerBuffer,
				uint16_t readLength, uint8_t* readBuffer);

decaIrqStatus_t decamutexon(void)
{
	// TODO
	return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
	// TODO
}

void deca_sleep(unsigned int time_ms)
{
	k_msleep(time_ms);
}

void deca_usleep(unsigned long time_us)
{
	k_usleep(time_us);
}

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = dw3000_readfromspi,
    .writetospi = dw3000_writetospi,
    .writetospiwithcrc = dw3000_writetospiwithcrc,
    .setslowrate = dw3000_spi_speed_slow,
    .setfastrate = dw3000_spi_speed_fast,
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = dw3000_wakeup,
};
