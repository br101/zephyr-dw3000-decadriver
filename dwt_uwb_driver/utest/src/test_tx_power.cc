/*
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <stdio.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

extern "C"
{
#include "deca_interface.h"
#include "deca_device_api.h"
#include "dw3000_deca_regs.h"
}

extern const struct dwt_driver_s dw3000_driver;

void deca_usleep(unsigned long time_us)
{
	(void)time_us;
}

void deca_sleep(unsigned int time_ms)
{
	(void)time_ms;
}

decaIrqStatus_t decamutexon(void)
{
	return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
	(void)s;
}

static int32_t readfromspi(uint16_t header_length, uint8_t *header_buffer, uint16_t read_length,
			   uint8_t *read_buffer)
{
	(void)header_length;
	(void)header_buffer;
	(void)read_length;
	(void)read_buffer;
	return DWT_SUCCESS;
}
static int32_t writetospi(uint16_t header_length, const uint8_t *header_buffer,
			  uint16_t write_length, const uint8_t *write_buffer)
{
	(void)header_length;
	(void)header_buffer;
	(void)write_length;
	(void)write_buffer;
	return DWT_SUCCESS;
}
static int32_t writetospiwithcrc(uint16_t header_length, const uint8_t *header_buffer,
				 uint16_t write_length, const uint8_t *write_buffer, uint8_t crc8)
{
	(void)header_length;
	(void)header_buffer;
	(void)write_length;
	(void)write_buffer;
	(void)crc8;
	return DWT_SUCCESS;
}
static void setslowrate(void)
{
}
static void setfastrate(void)
{
}
static void wakeup_device_with_io(void)
{
}

const struct dwt_driver_s *drv_ptr[] = { &dw3000_driver };


struct dw_priv {
	uint8_t spicrc;
};

struct TestTxPower:public::testing::Test {
    public:
	void SetUp() override
	{
		spi.readfromspi = readfromspi;
		spi.writetospi = writetospi;
		spi.setfastrate = setfastrate;
		spi.setslowrate = setslowrate;
		spi.writetospiwithcrc = writetospiwithcrc;

		probe_interf.dw = &dw;
		probe_interf.spi = &spi;
		probe_interf.wakeup_device_with_io = wakeup_device_with_io;
		probe_interf.driver_list = (struct dwt_driver_s **)drv_ptr;
		probe_interf.dw_driver_num = 1; /* Only one driver at a time. */

		dwt_probe(&probe_interf);
		/* Probe will fail because readfromspi is not mocked-up.
		 * But ignore the error and force dwt_driver to be dw3000_driver. */
		dw.dwt_driver = (struct dwt_driver_s *)&dw3000_driver;
		dw.priv = &priv;
	}

    protected:
	struct dwt_probe_s probe_interf;
	struct dwchip_s dw;
	struct dwt_spi_s spi;
	struct dw_priv priv;
};


TEST_F(TestTxPower, WhenExactTxPowerIsFoundInTable_Success)
{
	int channel = DWT_CH5;
	uint32_t exp_tx_power = 0xc2c2c2c2;
	uint8_t tx_power = 0xc2;
	uint8_t tx_power_idx;
	uint32_t pll_bias_trim;
	power_indexes_t p_indexes;
	tx_adj_res_t p_res;

	/* Expected returned values. */
	uint8_t exp_tx_power_idx = 5;
	uint8_t exp_pll_bias_trim = 7;

	int r = dwt_convert_tx_power_to_index(channel, tx_power, &tx_power_idx);
	ASSERT_EQ(r, DWT_SUCCESS);
	ASSERT_EQ(tx_power_idx, exp_tx_power_idx);

	for (int i = 0; i < DWT_MAX_POWER_INDEX; i++)
		p_indexes.input[i] = tx_power_idx;

	r = dwt_calculate_linear_tx_setting(channel, &p_indexes, &p_res);
	ASSERT_EQ(r, DWT_SUCCESS);
	ASSERT_EQ(p_res.tx_frame_cfg.tx_power_setting, exp_tx_power);
	pll_bias_trim = (p_res.tx_frame_cfg.pll_cfg & PLL_COMMON_PLL_BIAS_TRIM_MASK)
		>> PLL_COMMON_PLL_BIAS_TRIM_BIT_OFFSET;
	ASSERT_EQ(pll_bias_trim, exp_pll_bias_trim);
}

/**
 * TestLLHWWithParams section
 * Parameters description in the tuple:
 * dwt_config_t [input] - DWT chip configuration.
 * struct mcps802154_hrp_uwb_params [input] - HRP UWB Params.
 * FrameDurationResult [input] - Frame Duration Result.
 */
class TestConvertTxPowerToIdx
	: public TestTxPower,
	  public testing::WithParamInterface<
		  std::tuple<int, uint8_t> > {
};

TEST_P(TestConvertTxPowerToIdx, DoTest)
{
	int channel = std::get<0>(GetParam());
	uint8_t tx_power = std::get<1>(GetParam());
	uint8_t tx_power_idx;
	power_indexes_t p_indexes;
	tx_adj_res_t p_res;
	uint32_t pll_bias_trim;

	int r = dwt_convert_tx_power_to_index(channel, tx_power, &tx_power_idx);
	ASSERT_EQ(r, DWT_SUCCESS);

	for (int i = 0; i < DWT_MAX_POWER_INDEX; i++)
		p_indexes.input[i] = tx_power_idx;

	r = dwt_calculate_linear_tx_setting(channel, &p_indexes, &p_res);
	ASSERT_EQ(r, DWT_SUCCESS);

	pll_bias_trim = (p_res.tx_frame_cfg.pll_cfg & PLL_COMMON_PLL_BIAS_TRIM_MASK)
		>> PLL_COMMON_PLL_BIAS_TRIM_BIT_OFFSET;
	
	printf("TxPowerIn 0x%02x Chan %d => Idx 0x%02x TxPowerOut 0x%02x Bias %d\n",
	       tx_power, channel, tx_power_idx, p_res.tx_frame_cfg.tx_power_setting, pll_bias_trim);
}

INSTANTIATE_TEST_SUITE_P(ch5, TestConvertTxPowerToIdx,
			 testing::Values(std::make_tuple(5, 0x5d),
					 std::make_tuple(5, 0x61),
					 std::make_tuple(5, 0x65),
					 std::make_tuple(5, 0x69),
					 std::make_tuple(5, 0x6d),
					 std::make_tuple(5, 0x71),
					 std::make_tuple(5, 0x75),
					 std::make_tuple(5, 0x79),
					 std::make_tuple(5, 0x7d),
					 std::make_tuple(5, 0x85),
					 std::make_tuple(5, 0x9d)));

INSTANTIATE_TEST_SUITE_P(ch9, TestConvertTxPowerToIdx,
			 testing::Values(std::make_tuple(9, 0x79),
					 std::make_tuple(9, 0x7d),
					 std::make_tuple(9, 0x85),
					 std::make_tuple(9, 0x91),
					 std::make_tuple(9, 0x95),
					 std::make_tuple(9, 0x99),
					 std::make_tuple(9, 0x9d),
					 std::make_tuple(9, 0xa1),
					 std::make_tuple(9, 0xa5),
					 std::make_tuple(9, 0xa9),
					 std::make_tuple(9, 0xad),
					 std::make_tuple(9, 0xb1),
					 std::make_tuple(9, 0xb5),
					 std::make_tuple(9, 0xb9),
					 std::make_tuple(9, 0xc1)));
