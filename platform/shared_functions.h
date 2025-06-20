/*! ----------------------------------------------------------------------------
 * @file    shared_functions.h
 * @brief   Global functions are found here
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */
#ifndef _SHARE_FUNC_
#define _SHARE_FUNC_

#ifdef __cplusplus
extern "C"
{
#endif

/* Power boost calculation service function defines*/
#define LUT_1000_200_US_NUM     33 /* Number of frames duration values for which look up table has corresponding dial back in units of 0.1dB*/
#define LUT_1000_200_US_STEP    25 /* Frame duration step in us between each index in LUT */
#define LUT_1000_200_US_MIN     200 /* Minimum frame duration characterised by LUT*/
#define LUT_1000_200_US_MIN_BST 0 /* Boost to apply when a frame is longer or equal to the maximum duration*/

#define LUT_200_70_US_NUM     14 /* Number of frames duration values for which look up table has corresponding dial back in units of 0.1dB*/
#define LUT_200_70_US_STEP    10 /* Frame duration step in us between each index in LUT */
#define LUT_200_70_US_MIN     70 /* Minimum frame duration characterised by LUT*/
#define LUT_200_70_US_MAX_BST 113 /* Total boost to apply when a frame is equal or shorter to the minimum duration*/

#define FRAME_DURATION_REF 1000 /* The reference duration for a frame is 1000us. Longer frame will have 0dB boost.*/

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn calculate_power_boost()
     *
     * @brief Calculation the allowed power boost for a frame_duration_us frame relatively to a 1ms frame.
     *
     * @param reg: uint16_t duration of frame..
     *
     * @return boost: the amount of boost in 0.1dB steps which is allowed when transmitting the frame_dur_us frame
     *                relatively to a 1ms frame. For example, if the frame duration is 500us, then relatively to 1ms,
     *                a 3dB boost is allowed, and the function will return 30.
     */
    uint8_t calculate_power_boost(uint16_t frame_duration_us);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn check_for_status_errors()
     *
     * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
     *
     * @param reg: uint32_t value representing the current status register value.
     * @param errors: pointer to a uint32_t buffer that contains the sum of different errors logged during program operation.
     *
     * @return none
     */
    void check_for_status_errors(uint32_t reg, uint32_t *errors);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn get_rx_delay_time_txpreamble()
     *
     * @brief This function is used to get a value to increase the delay timer by dependent on the current TX preamble length set.
     *
     * @param None
     *
     * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
     */
    uint32_t get_rx_delay_time_txpreamble(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn get_rx_delay_time_data_rate()
     *
     * @brief This function is used to get a value to increase the delay timer by dependent on the current data rate set.
     *
     * @param None
     *
     * @return delay_time - a uint32_t value indicating the required increase needed to delay the time by.
     */
    uint32_t get_rx_delay_time_data_rate(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn set_delayed_rx_time()
     *
     * @brief This function is used to set the delayed RX time before running dwt_rxenable()
     *
     * @param delay - This is a defined delay value (usually POLL_TX_TO_RESP_RX_DLY_UUS)
     * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
     *                         is called.
     *
     * @return None
     */
    void set_delayed_rx_time(uint32_t delay, dwt_config_t *config_options);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn set_resp_rx_timeout()
     *
     * @brief This function is used to set the RX timeout value
     *
     * @param delay - This is a defined delay value (usually RESP_RX_TIMEOUT_UUS)
     * @param config_options - pointer to dwt_config_t configuration structure that is in use at the time this function
     *                         is called.
     *
     * @return None
     */
    void set_resp_rx_timeout(uint32_t delay, dwt_config_t *config_options);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn resp_msg_get_ts()
     *
     * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
     *        least significant byte is at the lower address.
     *
     * @param  ts_field  pointer on the first byte of the timestamp field to get
     *         ts  timestamp value
     *
     * @return none
     */
    void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn get_tx_timestamp_u64()
     *
     * @brief Get the TX time-stamp in a 64-bit variable.
     *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
     *
     * @param  none
     *
     * @return  64-bit value of the read time-stamp.
     */
    uint64_t get_tx_timestamp_u64(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn get_rx_timestamp_u64()
     *
     * @brief Get the RX time-stamp in a 64-bit variable.
     *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
     *
     * @param  none
     *
     * @return  64-bit value of the read time-stamp.
     */
    uint64_t get_rx_timestamp_u64(void);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn final_msg_get_ts()
     *
     * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
     *        significant byte is at the lower address.
     *
     * @param  ts_field  pointer on the first byte of the timestamp field to read
     *         ts  timestamp value
     *
     * @return none
     */
    void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn final_msg_set_ts()
     *
     * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
     *        message, the least significant byte is at the lower address.
     *
     * @param  ts_field  pointer on the first byte of the timestamp field to fill
     *         ts  timestamp value
     *
     * @return none
     */
    void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @fn resp_msg_set_ts()
     *
     * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
     *        response message, the least significant byte is at the lower address.
     *
     * @param  ts_field  pointer on the first byte of the timestamp field to fill
     *         ts  timestamp value
     *
     * @return none
     */
    void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);

    /*! ------------------------------------------------------------------------------------------------------------------
     * @brief This function will continuously read the system status register until it matches the bits set in the mask
     *        input parameter. It will then exit the function.
     *        This is useful to use when waiting on particular events to occurs. For example, the user could wait for a
     *        good UWB frame to be received and/or no receive errors have occurred.
     *        The lower 32-bits of the system status register will be read in a while loop. Each iteration of the loop will check if a matching
     *        mask value for the higher 32-bits of the system status register is set. If the mask value is set in the higher 32-bits of the system
     *        status register, the function will return that value along with the last recorded value of the lower 32-bits of the system status
     *        register. Thus, the user should be aware that this function will not wait for high and low mask values to be set in both the low and high
     *        system status registers. Alternatively, the user can call this function to *only* check the higher or lower system status registers.
     *
     * input parameters
     * @param lo_result - A pointer to a uint32_t that will contain the final value of the system status register (lower 32 bits).
     *                    Pass in a NULL pointer to ignore returning this value.
     * @param hi_result - A pointer to a uint32_t that will contain the final value of the system status register (higher 32 bits).
     *                    Pass in a NULL pointer to ignore returning this value.
     * @param lo_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (lower 32 bits).
     *               Example values to use are as follows:
     *               DWT_INT_TXFRS_BIT_MASK - Wait for a TX frame to be sent.
     *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no reception errors.
     *               SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR - Wait for frame to be received and no receive timeout errors
     *                                                                                          and no reception errors.
     *               SYS_STATUS_RXFR_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_ND_RX_ERR - Wait for packet to be received and no receive timeout errors
     *                                                                                            and no reception errors.
     *                                                                                            These flags are useful when polling for STS Mode 3 (no data)
     *                                                                                            packets.
     *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
     * @param hi_mask - a uint32 mask value that is used to check for certain bits to be set in the system status register (higher 32 bits).
     *               Example values to use are as follows:
     *               SYS_STATUS_HI_CCA_FAIL_BIT_MASK - Check for CCA fail status.
     *               0 - The function will not wait for any bits in the system status register (lower 32 bits).
     *
     * return None
     */
    void waitforsysstatus(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask);

#ifdef __cplusplus
}
#endif

#endif
