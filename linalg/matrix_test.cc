#include "linalg/matrix.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace spartonautics::linalg::testing {

TEST(MatrixTest, Iterating) {
  constexpr Matrix3 kM =
      Matrix3::Data{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
  constexpr std::array<double, 9> kData = {1.0, 2.0, 3.0, 4.0, 5.0,
                                           6.0, 7.0, 8.0, 9.0};

  auto data_it = kData.cbegin();
  for (auto row_it = kM.cbegin(); row_it < kM.cend(); row_it++) {
    for (auto col_it = row_it->begin(); col_it < row_it->end(); col_it++) {
      EXPECT_EQ(*col_it, *data_it);
      data_it++;
    }
  }
  EXPECT_EQ(data_it, kData.end());

  data_it = kData.cbegin();
  for (auto row : kM.data()) {
    for (double scalar : row) {
      EXPECT_EQ(scalar, *data_it);
      data_it++;
    }
  }
  EXPECT_EQ(data_it, kData.end());
}

TEST(MatrixTest, AddSubtract) {
  constexpr Matrix3 kM1 =
      Matrix3::Data{{{1.0, 2.0, 3.0}, {3.0, 2.0, 1.0}, {-1.0, -2.0, 3.0}}};
  constexpr Matrix3 kM2 =
      Matrix3::Data{{{-1.0, 3.0, 2.0}, {1.0, 2.0, 2.0}, {3.0, -2.0, 2.0}}};
  EXPECT_EQ(kM1 + kM2,
            Matrix3(Matrix3::Data{
                {{0.0, 5.0, 5.0}, {4.0, 4.0, 3.0}, {2.0, -4.0, 5.0}}}));
  EXPECT_EQ(kM1 - kM2,
            Matrix3(Matrix3::Data{
                {{2.0, -1.0, 1.0}, {2.0, 0.0, -1.0}, {-4.0, 0.0, 1.0}}}));
}

TEST(MatrixTest, Multiply) {
  constexpr Matrix<2, 3> kM1 =
      Matrix<2, 3>::Data{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}};
  constexpr Matrix<3, 2> kM2 =
      Matrix<3, 2>::Data{{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}}};
  EXPECT_EQ(kM1 * kM2, Matrix2(Matrix2::Data{{{58.0, 64.0}, {139.0, 154.0}}}));
  constexpr auto kScaled = Matrix<2, 3>(
      Matrix<2, 3>::Data{{{-2.0, -4.0, -6.0}, {-8.0, -10.0, -12.0}}});
  constexpr auto kScaledResult = -2.0 * kM1;
  EXPECT_EQ(kScaledResult, kScaled);
}

TEST(MatrixTest, Transpose) {
  constexpr Matrix<3, 2> kM1 =
      Matrix<3, 2>::Data{{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}}};
  constexpr Matrix<2, 3> kM1Dagger =
      Matrix<2, 3>::Data{{{1.0, 3.0, 5.0}, {2.0, 4.0, 6.0}}};
  EXPECT_EQ(kM1.Transpose(), kM1Dagger);
  constexpr Matrix<1, 2> kM2 = Matrix<1, 2>::Data{{{1.0, 2.0}}};
  EXPECT_EQ(kM2.Transpose(), Vector2(Vector2::Data{1.0, 2.0}));
}

TEST(MatrixTest, Identity) {
  constexpr auto kI2 = Matrix2(Matrix2::Data{{{1.0, 0.0}, {0.0, 1.0}}});
  EXPECT_EQ(Matrix2::Identity(), kI2);
  constexpr auto kI3 = Matrix3(
      Matrix3::Data{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}});
  EXPECT_EQ(Matrix3::Identity(), kI3);
}

TEST(MatrixTest, Vector) {
  Vector3 v = Vector3::Data{{1.0, 2.0, 3.0}};
  EXPECT_EQ(v.x(), 1.0);
  EXPECT_EQ(v.y(), 2.0);
  EXPECT_EQ(v.z(), 3.0);

  v(1) = 4.0;
  EXPECT_EQ(v.y(), 4.0);

  EXPECT_DOUBLE_EQ(v.Norm(), std::sqrt(1.0 + 16.0 + 9.0));

  constexpr Vector3 kV2 = Vector3::Data{{-2.0, 1.0, 5.0}};
  EXPECT_DOUBLE_EQ(v.Dot(kV2), -2.0 + 4.0 + 15.0);
}

}  // namespace spartonautics::linalg::testing
