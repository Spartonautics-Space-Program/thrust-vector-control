#include "linalg/matrix.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace spartonautics::linalg::testing {

TEST(MatrixTest, Iterating) {
  constexpr Matrix3 kM =
      Matrix3::stl_matrix{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
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
}

TEST(MatrixTest, AddSubtract) {
  constexpr Matrix3 kM1 = Matrix3::stl_matrix{
      {{1.0, 2.0, 3.0}, {3.0, 2.0, 1.0}, {-1.0, -2.0, 3.0}}};
  constexpr Matrix3 kM2 = Matrix3::stl_matrix{
      {{-1.0, 3.0, 2.0}, {1.0, 2.0, 2.0}, {3.0, -2.0, 2.0}}};
  EXPECT_EQ(kM1 + kM2,
            Matrix3(Matrix3::stl_matrix{
                {{0.0, 5.0, 5.0}, {4.0, 4.0, 3.0}, {2.0, -4.0, 5.0}}}));
  EXPECT_EQ(kM1 - kM2,
            Matrix3(Matrix3::stl_matrix{
                {{2.0, -1.0, 1.0}, {2.0, 0.0, -1.0}, {-4.0, 0.0, 1.0}}}));
}

TEST(MatrixTest, Multiply) {
  constexpr Matrix<double, 2, 3> kM1 =
      Matrix<double, 2, 3>::stl_matrix{{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}};
  constexpr Matrix<double, 3, 2> kM2 =
      Matrix<double, 3, 2>::stl_matrix{{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}}};
  EXPECT_EQ(kM1 * kM2,
            Matrix2(Matrix2::stl_matrix{{{58.0, 64.0}, {139.0, 154.0}}}));
  constexpr auto kScaled =
      Matrix<double, 2, 3>(Matrix<double, 2, 3>::stl_matrix{
          {{-2.0, -4.0, -6.0}, {-8.0, -10.0, -12.0}}});
  constexpr auto kScaledResult = -2.0 * kM1;
  EXPECT_EQ(kScaledResult, kScaled);
}

TEST(MatrixTest, Vector) {
  Vector3 v = Vector3::stl_col{{1.0, 2.0, 3.0}};
  EXPECT_EQ(v.x(), 1.0);
  EXPECT_EQ(v.y(), 2.0);
  EXPECT_EQ(v.z(), 3.0);

  v(1, 0) = 4.0;
  EXPECT_EQ(v.y(), 4.0);
}

}  // namespace spartonautics::linalg::testing
