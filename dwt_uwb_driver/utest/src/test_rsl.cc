/*
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <gtest/gtest.h>

#include <cmath>

extern "C"
{
#include "deca_device_api.h"
#include "deca_rsl.h"
}

static double convert_q_to_double(int32_t x, uint8_t frac_part)
{
	return (double)x / (1 << frac_part);
}

static double
EstimateReceiveSignalPower(double power, double preamble_accumulation_count,
	uint8_t dgc_decision, double rssi_constant_dbm)
{
	double n_squared =
		preamble_accumulation_count * preamble_accumulation_count;

	return 10.0 * log10((power) /
			    n_squared) +
	       6 * dgc_decision - rssi_constant_dbm;
}

static double GetRssiConstant(uint8_t rx_code, bool sts)
{
	double constant_dbm;
	if (rx_code >= PCODE_PRF64_START) {
		constant_dbm = 121.7;
		if (sts) {
			constant_dbm -= 1.0;
		}
	} else {
		constant_dbm = 113.8;
	}
	return constant_dbm;
}

struct SignalPowerTestCase {
	uint32_t channel_impulse_response;
	uint8_t quantization_factor;
	uint16_t preamble_accumulation_count;
	double dgc_decision;
	uint8_t rx_pcode;
	bool is_sts;
	double expected_rssi_dbm;
};

class SignalPowerPrecisionTest
	: public ::testing::TestWithParam<SignalPowerTestCase> {
    public:
	static const SignalPowerTestCase test_cases[];
};

constexpr double invalid_q8 = -128.0;

const SignalPowerTestCase SignalPowerPrecisionTest::test_cases[] = {
	/*
	 * Dataset computed on a spreadsheet.
	 * RSSI constant for a PRF of 64 MHz Ipatov preamble with no STS.
	 * (constant is 121.7)
	 */
	{ 47, 21, 65, 0, 9, false, -78.0 },
	{ 45, 21, 65, 0, 9, false, -78.2 },
	{ 44, 21, 65, 0, 9, false, -78.3 },
	{ 44, 21, 65, 0, 9, false, -78.3 },
	{ 41, 21, 65, 0, 9, false, -78.6 },
	{ 40, 21, 65, 0, 9, false, -78.7 },
	{ 39, 21, 65, 0, 9, false, -78.8 },
	{ 37, 21, 65, 0, 9, false, -79.1 },
	{ 36, 21, 65, 0, 9, false, -79.2 },
	{ 32, 21, 65, 0, 9, false, -79.7 },
	{ 28, 21, 65, 0, 9, false, -80.3 },
	{ 23, 21, 65, 0, 9, false, -81.1 },
	{ 13, 21, 65, 0, 9, false, -83.6 },
	{ 18, 21, 65, 0, 9, false, -82.2 },
	{ 15, 21, 65, 0, 9, false, -83.0 },
	{ 13, 21, 65, 1, 9, false, -77.6 },
	{ 5, 21, 65, 0, 9, false, -87.8 },
	{ 2, 21, 65, 0, 9, false, -91.7 },
	/* 16 MHz PRF. */
	{ 2, 21, 65, 0, 8, false, -83.83 },
	/* Test max value for the power. */
	{ INT32_MAX, 0, 42, 1, 9, false, -54.85 },
	/*
	 * Values failing gracefully (result = -128 dBm).
	 */
	/* Power is 0. */
	{ 0, 0, 42, 1, 9, false, invalid_q8 },
	/* Preamble accumulation count (n) is 0. */
	{ 4, 0, 0, 1, 9, false, invalid_q8 },
	/* n² is not representable by an int32_t. */
	{ 4, 0, UINT16_MAX, 0, 9, false, invalid_q8 },
};

TEST_P(SignalPowerPrecisionTest, ForEachTestCase)
{
	SignalPowerTestCase params = GetParam();

	/* Compute RSSI using doubles, check it is near expected. */
	double constant_dbm =
		GetRssiConstant(params.rx_pcode, params.is_sts);
	double rssi_dbm = EstimateReceiveSignalPower(
		params.channel_impulse_response << params.quantization_factor,
		params.preamble_accumulation_count, params.dgc_decision,
		constant_dbm);
	if (params.expected_rssi_dbm != invalid_q8) {
		// Compare with floating point math result when the result
		// is supposed to be valid.
		EXPECT_NEAR(rssi_dbm, params.expected_rssi_dbm, 0.05);
	}

	/* Compute RSSI using fixed point, check percision is 0.1 dBm. */

	int16_t llhw_rsl_q8 = rsl_calculate_signal_power(
    	params.channel_impulse_response,
    	params.quantization_factor,
		params.preamble_accumulation_count,
    	params.dgc_decision,
    	params.rx_pcode,
    	params.is_sts
	);

	double llhw_rssi_dbm_f = convert_q_to_double(llhw_rsl_q8, 8);

	// Expected resolution 0.1 dBm (±0.05 dBm)
	EXPECT_NEAR(params.expected_rssi_dbm, llhw_rssi_dbm_f, 0.05);
}

INSTANTIATE_TEST_CASE_P(
	ForEachTestCase, SignalPowerPrecisionTest,
	::testing::ValuesIn(SignalPowerPrecisionTest::test_cases));

struct RxPowerTestCase {
	uint32_t power;
	uint32_t f1;
	uint32_t f2;
	uint32_t f3;
	uint16_t preamble_accumulation_count;
	uint8_t dgc_decision;
	uint8_t rx_code;
	bool sts;
	double expected_fp_rsl_dbm;
};

class TestLLHWRxPower : public ::testing::TestWithParam<RxPowerTestCase> {
    public:
	static const RxPowerTestCase test_cases[];
};

const RxPowerTestCase TestLLHWRxPower::test_cases[] = {
	/* Real world data, we should add some extreme values too */
	{ .power = 0xa,
	  .f1 = 0x26d9,
	  .f2 = 0x27ea,
	  .f3 = 0x159e,
	  .preamble_accumulation_count = 0x3f,
	  .dgc_decision = 3,
	  .rx_code = 10,
	  .sts = true,
	  .expected_fp_rsl_dbm = -67.04 },
};

TEST_P(TestLLHWRxPower, ForEachCase)
{
	/* Compute Rx power using doubles, check it is near expected. */

	RxPowerTestCase params = GetParam();
	double constant_dbm =
		GetRssiConstant(params.rx_code, params.sts);

	double f1 = convert_q_to_double(params.f1, 2);
	double f2 = convert_q_to_double(params.f2, 2);
	double f3 = convert_q_to_double(params.f3, 2);

	double fp_rsl_dbm = EstimateReceiveSignalPower(
				f1 * f1 + f2 * f2 + f3 * f3,
				params.preamble_accumulation_count,
				params.dgc_decision, constant_dbm);

	EXPECT_NEAR(fp_rsl_dbm, params.expected_fp_rsl_dbm, 0.05);

	int16_t llhw_fp_rsl_q8 = rsl_calculate_first_path_power(
		params.f1,
		params.f2,
		params.f3,
		params.preamble_accumulation_count,
		params.dgc_decision,
		params.rx_code,
		params.sts
	);

	// Expected resolution 0.1 dBm (±0.05 dBm)
	EXPECT_NEAR(params.expected_fp_rsl_dbm,
		    convert_q_to_double(llhw_fp_rsl_q8, 8), 0.05);
}

INSTANTIATE_TEST_CASE_P(ForEachCase, TestLLHWRxPower,
			::testing::ValuesIn(TestLLHWRxPower::test_cases));