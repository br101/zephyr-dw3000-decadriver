#include <kernel.h>

#include "deca_device_api.h"

/* This file implements the functions required by decadriver */

void wakeup_device_with_io(void)
{
}

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
