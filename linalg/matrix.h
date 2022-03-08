#ifndef MATRIX_H_
#define MATRIX_H_

#include <array>
#include <cmath>
#include <iostream>

namespace spartonautics::linalg {

// Fixed size, stack-allocated dense matrix
template <typename S, size_t Rows, size_t Cols>
class Matrix {
 public:
  using Row = std::array<S, Cols>;
  using Data = std::array<Row, Rows>;

  // Zeros the matrix by default
  constexpr Matrix() : data_() {
    for (auto &row : data_) {
      for (auto &scalar : row) {
        scalar = S(0.0);
      }
    }
  }
  constexpr Matrix(const Data &mat) : data_(mat) {}

  virtual void Print(std::ostream &os) const {
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

  constexpr Data *mutable_data() { return data_; }
  constexpr const Data &data() const { return data_; }

  constexpr S &operator()(const size_t i, const size_t j) {
    return data_[i][j];
  }
  constexpr Row &operator[](const size_t i) const { return data_[i]; }
  constexpr S x() { return at(0, 0); }
  constexpr S y() { return at(1, 0); }
  constexpr S z() { return at(2, 0); }

  constexpr const Row &row(const size_t i) const { return data_[i]; }
  constexpr const S &at(const size_t i, const size_t j) const {
    return data_[i][j];
  }
  constexpr S x() const { return at(0, 0); }
  constexpr S y() const { return at(1, 0); }
  constexpr S z() const { return at(2, 0); }

  constexpr Data::iterator begin() { return data_.begin(); }
  constexpr Data::const_iterator cbegin() const { return data_.cbegin(); }
  constexpr Data::iterator end() { return data_.end(); }
  constexpr Data::const_iterator cend() const { return data_.cend(); }

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

 protected:
  Data data_;

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
};

template <typename S, size_t Size>
class Vector : public Matrix<S, Size, 1> {
 public:
  using Data = std::array<S, Size>;

  constexpr Vector() : Matrix<S, Size, 1>() {}
  constexpr Vector(const Data &vec) {
    for (size_t i = 0; i < Size; i++) {
      Matrix<S, Size, 1>::data_[i][0] = vec[i];
    }
  }

  virtual void Print(std::ostream &os) const override {
    os << '[';
    for (size_t i = 0; i < Size - 1; i++) {
      os << Matrix<S, Size, 1>::at(i, 0) << ", ";
    }
    os << Matrix<S, Size, 1>::at(Size - 1, 0) << ']';
  }

  constexpr S &operator()(const size_t i) {
    return Matrix<S, Size, 1>::data_[i][0];
  }

  constexpr const S &at(const size_t i) const {
    return Matrix<S, Size, 1>::at(i, 0);
  }

  constexpr S Dot(const Vector<S, Size> &vec) const {
    S dot = S(0.0);
    for (size_t i = 0; i < Size; i++) {
      dot += at(i) * vec.at(i);
    }
    return dot;
  }

  constexpr S Norm() const {
    S norm = S(0.0);
    for (size_t i = 0; i < Size; i++) {
      norm += std::pow(at(i), S(2.0));
    }
    return std::sqrt(norm);
  }
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
using Vector2 = Vector<double, 2>;
using Vector3 = Vector<double, 3>;
using Vector4 = Vector<double, 3>;

}  // namespace spartonautics::linalg

#endif  // MATRIX_H_
