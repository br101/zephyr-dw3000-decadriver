#ifndef DW3000_H

int dw3000_init(void);
int dw3000_init_interrupt(void);
void dw3000_fini(void);
void dw3000_hw_reset(void);
bool dw3000_wakeup_pin(void);

void dw3000_spi_speed_slow(void);
void dw3000_spi_speed_fast(void);

#endif
