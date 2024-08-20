#include <zephyr/kernel.h>

#include "deca_interface.h"

#include "dw3000_hw.h"
#include "dw3000_spi.h"

/* This file implements the functions required by decadriver */

decaIrqStatus_t decamutexon(void)
{
	dw3000_hw_interrupt_disable();
	return 1;
}

void decamutexoff(decaIrqStatus_t s)
{
	// TODO?: s is not used
	dw3000_hw_interrupt_enable();
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
	.readfromspi = dw3000_spi_read,
	.writetospi = dw3000_spi_write,
	.writetospiwithcrc = dw3000_spi_write_crc,
	.setslowrate = dw3000_spi_speed_slow,
	.setfastrate = dw3000_spi_speed_fast,
};

const struct dwt_probe_s dw3000_probe_interf = {
	.dw = NULL,
	.spi = (void*)&dw3000_spi_fct,
	.wakeup_device_with_io = dw3000_hw_wakeup,
};
