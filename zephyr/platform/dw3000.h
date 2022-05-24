#ifndef DW3000_H

int dw3000_init(void);
int dw3000_init_interrupt(void);
void dw3000_fini(void);
void dw3000_hw_reset(void);
void dw3000_wakeup(void);
void dw3000_wakeup_pin_low(void);

void dw3000_spi_speed_slow(void);
void dw3000_spi_speed_fast(void);

#endif
