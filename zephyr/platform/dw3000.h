#ifndef DW3000_H

typedef void (*dw3000_isr_t)(void);

int dw3000_init(void);
int dw3000_set_isr(dw3000_isr_t isr);
void dw3000_fini(void);
void dw3000_hw_reset(void);

void dw3000_spi_speed_slow(void);
void dw3000_spi_speed_fast(void);

#endif
