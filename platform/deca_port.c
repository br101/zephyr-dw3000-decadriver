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

#if CONFIG_DW3000_CHIP_DW3000
extern const struct dwt_driver_s dw3000_driver;
#elif CONFIG_DW3000_CHIP_DW3720
extern const struct dwt_driver_s dw3720_driver;
#endif

const struct dwt_driver_s* tmp_ptr[] = {
#if CONFIG_DW3000_CHIP_DW3000
	&dw3000_driver,
#elif CONFIG_DW3000_CHIP_DW3720
	&dw3720_driver
#endif
};

const struct dwt_probe_s dw3000_probe_interf = {
	.dw = NULL,
	.spi = (void*)&dw3000_spi_fct,
	.wakeup_device_with_io = dw3000_hw_wakeup,
	.driver_list = (struct dwt_driver_s**)tmp_ptr,
	.dw_driver_num = 1,
};
