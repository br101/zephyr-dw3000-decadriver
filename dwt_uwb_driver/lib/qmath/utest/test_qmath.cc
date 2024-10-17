/*
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <gtest/gtest.h>

#include <cmath>

extern "C"
{
#include "qmath.h"
}

class TestPowOfBase10 : public ::testing::TestWithParam<int>
{
};

INSTANTIATE_TEST_SUITE_P(PowerOfBase2Data, TestPowOfBase10, testing::Range(-40, 40, 1));

TEST_P(TestPowOfBase10, fullRange)
{
    double param = GetParam();
    float real_val;
    uint32_t result_q8 = q8_pow_of_base2(param * LOG2_10_DIV_10_Q16);

    real_val = pow(10, param * 0.25 / 10);

    EXPECT_NEAR(result_q8 / 256.0f, real_val, 0.17);
}

class TestLogInt : public ::testing::TestWithParam<int>
{
};

INSTANTIATE_TEST_SUITE_P(LogData, TestLogInt, testing::Range(1, 1000000, 15000));

TEST_P(TestLogInt, logx)
{
    uint32_t param = GetParam();
    double log_val_q8 = 10 * log10(param);
    uint32_t result_q8 = log10_10(param);

    EXPECT_NEAR(result_q8 / 100.0, log_val_q8, 0.15);
}

class TestLogMax : public ::testing::Test
{
};

TEST_F(TestLogMax, logMax)
{
    uint32_t param = 0xffffffff;
    double log_val_q8 = 10 * log10(param);
    uint32_t result_q8 = log10_10(param);

    EXPECT_NEAR(result_q8 / 100.0, log_val_q8, 0.15);
}

class TestLogMin : public ::testing::Test
{
};

TEST_F(TestLogMin, logMin)
{
    uint32_t param = 0;
    uint32_t result_q8 = log10_10(param);

    EXPECT_EQ(result_q8, LOG_INVALID_VALUE);
}
