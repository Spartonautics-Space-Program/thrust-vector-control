#ifndef MATRIX_H_
#define MATRIX_H_

#include <array>
#include <iostream>

namespace spartonautics::linalg {

template <typename S, size_t Rows, size_t Cols>
class Matrix {
 public:
  typedef std::array<S, Cols> stl_row;
  typedef std::array<S, Rows> stl_col;
  typedef std::array<stl_row, Rows> stl_matrix;

  // Zeros the matrix by default
  constexpr Matrix() : data_() {
    for (auto &row : data_) {
      row.fill(S(0.0));
    }
  }
  constexpr Matrix(const stl_matrix &mat) : data_(mat) {}
  constexpr Matrix(const stl_col &vector) : Matrix() {
    for (size_t i = 0; i < Rows; i++) {
      data_[i][0] = vector[i];
    }
  }

  void Print(std::ostream &os) const {
    if (Cols == 1) {  // Vector
      os << '[';
      for (size_t i = 0; i < Rows - 1; i++) {
        os << at(i, 0) << ", ";
      }
      os << at(Rows - 1, 0) << ']';
    } else {  // Matrix
      os << '[';
      for (size_t i = 0; i < Rows; i++) {
        os << '[';
        for (size_t j = 0; j < (Cols - 1); j++) {
          os << at(i, j) << ", ";
        }
        os << at(i, Cols - 1) << ']';
        if (i != Rows - 1) {
          os << std::endl;
        }
      }
      os << ']';
    }
  }

  constexpr stl_matrix &data() { return data_; }
  constexpr const stl_matrix &data() const { return data_; }

  constexpr S &operator()(const size_t i, const size_t j) {
    return data_[i][j];
  }
  constexpr stl_row &operator[](const size_t i) const { return data_[i]; }
  constexpr S x() { return at(0, 0); }
  constexpr S y() { return at(1, 0); }
  constexpr S z() { return at(2, 0); }

  constexpr const stl_row &row(const size_t i) const { return data_[i]; }
  constexpr const S &at(const size_t i, const size_t j) const {
    return data_[i][j];
  }
  constexpr S x() const { return at(0, 0); }
  constexpr S y() const { return at(1, 0); }
  constexpr S z() const { return at(2, 0); }

  constexpr stl_matrix::iterator begin() { return data_.begin(); }
  constexpr stl_matrix::const_iterator cbegin() const { return data_.cbegin(); }
  constexpr stl_matrix::iterator end() { return data_.end(); }
  constexpr stl_matrix::const_iterator cend() const { return data_.cend(); }

  constexpr bool operator==(const Matrix<S, Rows, Cols> &mat) const {
    bool equal = true;
    for (size_t i = 0; equal && i < Rows; i++) {
      for (size_t j = 0; equal && j < Cols; j++) {
        if (at(i, j) != mat.at(i, j)) {
          equal = false;
        }
      }
    }
    return equal;
  }
  constexpr bool operator!=(const Matrix<S, Rows, Cols> &mat) const {
    return !(*this == mat);
  }

  constexpr void operator+=(const Matrix<S, Rows, Cols> &mat) {
    AddSubtract(true, mat, this);  // true means addition
  }
  constexpr void operator-=(const Matrix<S, Rows, Cols> &mat) {
    AddSubtract(false, mat, this);  // false means subtraction
  }
  constexpr Matrix<S, Rows, Cols> operator+(
      const Matrix<S, Rows, Cols> &mat) const {
    Matrix<S, Rows, Cols> sum;
    AddSubtract(true, mat, &sum);
    return sum;
  }
  constexpr Matrix<S, Rows, Cols> operator-(
      const Matrix<S, Rows, Cols> &mat) const {
    Matrix<S, Rows, Cols> diff;
    AddSubtract(false, mat, &diff);
    return diff;
  }

  template <size_t MCols>
  constexpr Matrix<S, Rows, MCols> operator*(
      const Matrix<S, Cols, MCols> &mat) const {
    Matrix<S, Rows, MCols> product;
    for (size_t row = 0; row < Rows; row++) {
      for (size_t col = 0; col < MCols; col++) {
        S sum = 0;
        for (size_t i = 0; i < Cols; i++) {
          sum += at(row, i) * mat.at(i, col);
        }
        product(row, col) = sum;
      }
    }
    return product;
  }

  constexpr void operator*=(const S scalar) { Scale(scalar, this); }
  constexpr Matrix<S, Rows, Cols> operator*(const S scalar) const {
    Matrix<S, Rows, Cols> scaled;
    Scale(scalar, &scaled);
    return scaled;
  }

 private:
  constexpr void AddSubtract(const bool addition,
                             const Matrix<S, Rows, Cols> &src,
                             Matrix<S, Rows, Cols> *dst) const {
    for (size_t i = 0; i < Rows; i++) {
      for (size_t j = 0; j < Cols; j++) {
        (*dst)(i, j) =
            (addition ? at(i, j) + src.at(i, j) : at(i, j) - src.at(i, j));
      }
    }
  }

  constexpr void Scale(const S scalar, Matrix<S, Rows, Cols> *dst) const {
    for (size_t i = 0; i < Rows; i++) {
      for (size_t j = 0; j < Cols; j++) {
        (*dst)(i, j) = at(i, j) * scalar;
      }
    }
  }

  stl_matrix data_;
};

template <typename S, size_t Rows, size_t Cols>
std::ostream &operator<<(std::ostream &os, const Matrix<S, Rows, Cols> &mat) {
  mat.Print(os);
  return os;
}

template <typename S, size_t Rows, size_t Cols>
constexpr Matrix<S, Rows, Cols> operator*(const S scalar,
                                          const Matrix<S, Rows, Cols> &mat) {
  return mat * scalar;
}

using Matrix2 = Matrix<double, 2, 2>;
using Matrix3 = Matrix<double, 3, 3>;
using Matrix4 = Matrix<double, 4, 4>;
using Vector2 = Matrix<double, 2, 1>;
using Vector3 = Matrix<double, 3, 1>;
using Vector4 = Matrix<double, 3, 1>;

}  // namespace spartonautics::linalg

#endif  // MATRIX_H_
