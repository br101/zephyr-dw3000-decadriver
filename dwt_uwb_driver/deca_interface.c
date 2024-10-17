/**
 * @file      deca_interface.c
 *
 * @brief     implementation of the common interface fn()
 *            if driver require different implementation, it can override it with a
 *            static fn() in the dwXXXX_device.c
 *
 * @author    Decawave Applications
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#include "deca_interface.h"
#include "deca_device_api.h"
#include <stdbool.h>

#ifndef AUTO_DW3300Q_DRIVER
/**
 * coex_gpio - change the state of the GPIO used for WiFi coexistence
 * @dw: the DW device
 * @state: the new GPIO state to set
 * @delay_us: delay before toggling the GPIO.
 *
 * This function only call the version dependent coex_gpio function.
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int32_t coex_gpio(struct dwchip_s *dw, int32_t state, int32_t delay_us)
{
    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;
    uint16_t val;
    int32_t retVal;

    if (delay_us != 0)
    {
        deca_usleep((uint32_t)delay_us);
    }

    (void)ops->ioctl(dw, DWT_READGPIOVALUE, 0, &val);
    if (state == dw->coex_gpio_active_state)
    {
        val |= ((uint16_t)1U << (uint16_t)dw->coex_gpio_pin);
        retVal = ops->ioctl(dw, DWT_SETGPIOVALUE, 0, &val);
    }
    else
    {
        val &= ~((uint16_t)1U << (uint16_t)dw->coex_gpio_pin);
        retVal =  ops->ioctl(dw, DWT_SETGPIOVALUE, 0, &val);
    }

    return retVal;
}

/**
 * coex_start - Handle WiFi coex gpio at start of uwb exchange.
 * @dw: the DW device
 * @trx_delayed: pointer to tx/rx_delayed parameter to update
 * @trx_date_dtu: pointer to tx/rx_date_dtu parameter to update
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int32_t coex_start(struct dwchip_s *dw, int32_t *trx_delayed, uint32_t *trx_date_dtu)
{
    uint32_t cur_time_dtu;
    uint8_t buf[4];
    int32_t delay_us, ret = 0;

    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;

    if (dw->coex_gpio_pin >= 0)
    {
        /* Read current DTU time to compare with next transfer time */
        ret = ops->ioctl(dw, DWT_READSYSTIME, 0, &buf);
        if (ret == 0)
        {
            cur_time_dtu = (((uint32_t)buf[3] << 24UL) | ((uint32_t)buf[2] << 16UL) | ((uint32_t)buf[1] << 8UL) | (uint32_t)buf[0]);

            if (*trx_delayed == 0)
            {
                /* Set gpio now */
                delay_us = 0;
                /* Change to delayed TX/RX with a 1ms delay */
                *trx_date_dtu = cur_time_dtu + COEX_TIME_DTU + COEX_MARGIN_DTU;
                *trx_delayed = 1;
            }
            else
            {
                /* Calculate when we need to toggle the gpio */
                int32_t time_difference_dtu = (int32_t)((int64_t)*trx_date_dtu - (int64_t)cur_time_dtu);
                int32_t time_difference_us = DTU_TO_US_S(time_difference_dtu);
                if (time_difference_us <= ((int32_t)COEX_TIME_US + (int32_t)COEX_MARGIN_US))
                {
                    delay_us = 0;
                }
                else
                {
                    delay_us = time_difference_us - (int32_t)COEX_TIME_US - (int32_t)COEX_MARGIN_US;
                }
            }
            /* Set coexistence gpio on chip */
            ret = coex_gpio(dw, 1, delay_us);
        }
    }

    return ret;
}

/**
 * coex_stop - Handle WiFi coex gpio at end of uwb exchange.
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int32_t coex_stop(struct dwchip_s *dw)
{
    int32_t ret = 0;

    if (dw->coex_gpio_pin >= 0)
    {
        /* Reset coex GPIO on chip */
        ret = coex_gpio(dw, 0, 0);
    }

    return ret;
}
#endif /* AUTO_DW3300Q_DRIVER */

int32_t interface_tx_frame(struct dwchip_s *dw, uint8_t *data, size_t len, struct dw_tx_frame_info_s *info)
{
    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;

    if (len != 0UL)
    {
        struct dwt_rw_data_s wr = { data, (uint16_t)len, 0U };
        (void)ops->ioctl(dw, DWT_WRITETXDATA, 0, (void *)&wr);

        struct dwt_tx_fctrl_s txfctrl = { (uint16_t)len, 0U, (((info->flag & MCPS_RANGING_BIT) != 0U) ? 1U : 0U) };
        (void)ops->ioctl(dw, DWT_WRITETXFCTRL, 0, (void *)&txfctrl); // Ranging bit shall not be set for non-ranging frames
    }

    struct dwt_enable_auto_ack_s ack = {
        .responseDelayTime = 0U, // local->ack_time,
        .enable = 0             /*local->ack_time << DW3000_ACK_RESP_ACK_TIM_BIT_OFFSET | info->rx_delay_dly; */
    };

    if (ack.enable != 0)
    {
        (void)ops->ioctl(dw, DWT_ENABLEAUTOACK, 0, (void *)&ack);
    }

    // Setup for delayed Transmit time (units are 4ns)
    if ((info->flag & ((uint32_t)DWT_START_TX_DELAYED | (uint32_t)DWT_START_TX_DLY_REF |
                       (uint32_t)DWT_START_TX_DLY_RS | (uint32_t)DWT_START_TX_DLY_TS)) != 0UL)
    {
        (void)ops->ioctl(dw, DWT_SETDELAYEDTRXTIME, 0, (void *)&info->tx_date_dtu);
    }

    // Setup for delayed Receive after Tx (units are sy = 1.0256 us)
    if (info->rx_delay_dly >= 0)
    {
        (void)ops->ioctl(dw, DWT_SETRXAFTERTXDELAY, 0, (void *)&info->rx_delay_dly);
        (void)ops->ioctl(dw, DWT_SETRXTIMEOUT, 0, (void *)&info->rx_timeout_pac);
    }

    return ops->ioctl(dw, DWT_STARTTX, 0, (void *)&info->flag);
}

uint64_t interface_get_timestamp(struct dwchip_s *dw)
{
    uint64_t ts = 0ULL;
    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;
    (void)ops->ioctl(dw, DWT_READRXTIMESTAMP, 0, (void *)&ts);
    return ts;
}

int32_t interface_rx_disable(struct dwchip_s *dw)
{
    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;
    return ops->ioctl(dw, DWT_FORCETRXOFF, 0, NULL);
}

int32_t interface_rx_enable(struct dwchip_s *dw, struct dw_rx_frame_info_s *info)
{
    const struct dwt_mcps_ops_s *ops = dw->dwt_driver->dwt_mcps_ops;
    int32_t rx_delayed = info->rx_delayed;
    uint32_t date_dtu = info->rx_date_dtu;
    uint32_t timeout_pac = info->rx_timeout_pac;
    int32_t error = ops->ioctl(dw, DWT_SETPREAMBLEDETECTTIMEOUT, 0, (void *)&timeout_pac);

    if (error == 0)
    {
#ifndef AUTO_DW3300Q_DRIVER
        /* Update RX parameters according to Wifi coexistence */
        /* this function may update rx_delayed and date_dtu */
        (void)coex_start(dw, &rx_delayed, &date_dtu);
#endif
        if (rx_delayed == 0)
        {
            error = ops->ioctl(dw, DWT_RXENABLE, (int32_t)DWT_START_RX_IMMEDIATE, NULL);
#ifndef AUTO_DW3300Q_DRIVER
            if (error != 0)
            {
                (void)coex_stop(dw);
            }
#endif
        }
        else
        {
            error = ops->ioctl(dw, DWT_SETDELAYEDTRXTIME, 0, (void *)&date_dtu);
            if (error == 0)
            {
                uint32_t rx_delayed_mask = ((uint32_t)DWT_START_RX_DELAYED | (uint32_t)DWT_IDLE_ON_DLY_ERR);
                error = ops->ioctl(dw, DWT_RXENABLE, (int32_t)rx_delayed_mask, NULL);
            }
#ifndef AUTO_DW3300Q_DRIVER
            else
            {
                (void)coex_stop(dw);
            }
#endif
        }
    }

    return error;
}

void interface_read_rx_frame(struct dwchip_s *dw, uint8_t *ptr, size_t len)
{
    struct dwt_rw_data_s tmp = { .buffer = ptr, .length = (uint16_t)len, .offset = 0U };

    (void)dw->dwt_driver->dwt_mcps_ops->ioctl(dw, DWT_READRXDATA, 0, &tmp);
    return;
}
