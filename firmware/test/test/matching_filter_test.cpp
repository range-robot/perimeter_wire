
#include <gtest/gtest.h>

// redefine ASF assert macro
#define ASSERT(cond) EXPECT_TRUE(cond)

#include "matching_filter.h"

int16_t samples1[] =
{
  0, 1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1,
  0, -1, -2, -3, -4, -5, -6, -7, -8, -7, -6, -5, -4, -3, -2, -1,
  0, 1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1,
  0, -1, -2, -3, -4, -5, -6, -7, -8, -7, -6, -5, -4, -3, -2, -1,
};

TEST(MatchingFilterTest, TestCrossCorrelation)
{
  // three equal strides
  int16_t buffer[sizeof(samples1) * 3];
  for (int i = 0; i < sizeof(samples1); i++)
  {
    buffer[i * 3 + 0] =
    buffer[i * 3 + 1] =
    buffer[i * 3 + 2] = samples1[i];
  }

  int8_t coeffs[] =
  {
    0, 1, 0, -1
  };

  // strides
  int16_t cc;
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer + 0, 1, 1);
  ASSERT_EQ(cc, -2);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer + 1, 1, 1);
  ASSERT_EQ(cc, -2);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer + 2, 1, 1);
  ASSERT_EQ(cc, -2);

  // repeat
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer, 2, 1);
  ASSERT_EQ(cc, -4);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer, 3, 1);
  ASSERT_EQ(cc, -2);

  // subsample
  cc = crossCorrelation(coeffs, 1, buffer, 1, 2);
  ASSERT_EQ(cc, 0);
  cc = crossCorrelation(coeffs, 2, buffer, 1, 2);
  ASSERT_EQ(cc, 5);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer, 1, 2);
  ASSERT_EQ(cc, -8);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer, 1, 8);
  ASSERT_EQ(cc, 72);
  cc = crossCorrelation(coeffs, sizeof(coeffs), buffer, 2, 8);
  ASSERT_EQ(cc, 144);
}


TEST(MatchingFilterTest, TestMatchingFilter)
{
  // three equal strides
  int16_t buffer[sizeof(samples1) * 3];
  for (int i = 0; i < sizeof(samples1); i++)
  {
    buffer[i * 3 + 0] =
    buffer[i * 3 + 1] =
    buffer[i * 3 + 2] = samples1[i];
  }

  int8_t coeffs[] =
  {
    0, 1, 0, -1
  };

  uint16_t q;
  int16_t sum;

  sum = corrFilter(coeffs, sizeof(coeffs), 1, 1, buffer, 16, &q);
  ASSERT_TRUE(sum == -2 || sum == 2);

  sum = corrFilter(coeffs, sizeof(coeffs), 8, 1, buffer, 32, &q);
  ASSERT_EQ(sum, -96);
  sum = corrFilter(coeffs, sizeof(coeffs), 8, 1, buffer+1, 32, &q);
  ASSERT_EQ(sum, -96);
}


TEST(MatchingFilterTest, TestMatchingFilterSync)
{
  // three equal strides
  int16_t buffer[sizeof(samples1) * 3];
  for (int i = 0; i < sizeof(samples1); i++)
  {
    buffer[i * 3 + 0] =
    buffer[i * 3 + 1] =
    buffer[i * 3 + 2] = samples1[i];
  }

  int8_t coeffs[] =
  {
    0, 1, 0, -1
  };

  uint16_t pos;
  uint32_t norm;
  int16_t mag[3];

  norm = corrFilterSync(coeffs, sizeof(coeffs), 1, 1, buffer, 16 * 3, mag, &pos);
  ASSERT_TRUE(mag[0] == -2 || mag[0] == 2);
  ASSERT_TRUE(mag[1] == -2 || mag[1] == 2);
  ASSERT_TRUE(mag[2] == -2 || mag[2] == 2);

  norm = corrFilterSync(coeffs, sizeof(coeffs), 8, 1, buffer, 32 * 3, mag, &pos);
  ASSERT_EQ(mag[0], -96);
  ASSERT_EQ(mag[1], -96);
  ASSERT_EQ(mag[2], -96);
  ASSERT_EQ(pos, 38);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

