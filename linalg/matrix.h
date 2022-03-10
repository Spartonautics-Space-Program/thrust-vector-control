#ifndef MATRIX_H_
#define MATRIX_H_

#include <array>
#include <cmath>
#include <iostream>

namespace spartonautics::linalg {

// Fixed size, stack-allocated dense matrix
template <size_t Rows, size_t Cols, typename S = double>
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

  constexpr bool operator==(const Matrix<Rows, Cols, S> &mat) const {
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
  constexpr bool operator!=(const Matrix<Rows, Cols, S> &mat) const {
    return !(*this == mat);
  }

  constexpr void operator+=(const Matrix<Rows, Cols, S> &mat) {
    AddSubtract(true, mat, this);  // true means addition
  }
  constexpr void operator-=(const Matrix<Rows, Cols, S> &mat) {
    AddSubtract(false, mat, this);  // false means subtraction
  }
  constexpr Matrix<Rows, Cols, S> operator+(
      const Matrix<Rows, Cols, S> &mat) const {
    Matrix<Rows, Cols, S> sum;
    AddSubtract(true, mat, &sum);
    return sum;
  }
  constexpr Matrix<Rows, Cols, S> operator-(
      const Matrix<Rows, Cols, S> &mat) const {
    Matrix<Rows, Cols, S> diff;
    AddSubtract(false, mat, &diff);
    return diff;
  }

  template <size_t Rows1, size_t Cols1, size_t Cols2>
  static constexpr void Multiply(const Matrix<Rows1, Cols1, S> &mat1,
                                 const Matrix<Cols1, Cols2, S> &mat2,
                                 Matrix<Rows1, Cols2, S> *product) {
    for (size_t row = 0; row < Rows1; row++) {
      for (size_t col = 0; col < Cols2; col++) {
        S sum = 0;
        for (size_t i = 0; i < Cols1; i++) {
          sum += mat1.at(row, i) * mat2.at(i, col);
        }
        (*product)(row, col) = sum;
      }
    }
  }

  template <size_t MCols>
  constexpr Matrix<Rows, MCols, S> operator*(
      const Matrix<Cols, MCols, S> &mat) const {
    Matrix<Rows, MCols, S> product;
    Multiply(*this, mat, &product);
    return product;
  }

  constexpr void operator*=(const S scalar) { Scale(scalar, this); }
  constexpr Matrix<Rows, Cols, S> operator*(const S scalar) const {
    Matrix<Rows, Cols, S> scaled;
    Scale(scalar, &scaled);
    return scaled;
  }

  constexpr Matrix<Cols, Rows, S> Transpose() const {
    Matrix<Cols, Rows, S> dagger;
    for (size_t i = 0; i < Rows; i++) {
      for (size_t j = 0; j < Cols; j++) {
        dagger(j, i) = at(i, j);
      }
    }
    return dagger;
  }

 protected:
  Data data_;

 private:
  constexpr void AddSubtract(const bool addition,
                             const Matrix<Rows, Cols, S> &src,
                             Matrix<Rows, Cols, S> *dst) const {
    for (size_t i = 0; i < Rows; i++) {
      for (size_t j = 0; j < Cols; j++) {
        (*dst)(i, j) =
            (addition ? at(i, j) + src.at(i, j) : at(i, j) - src.at(i, j));
      }
    }
  }

  constexpr void Scale(const S scalar, Matrix<Rows, Cols, S> *dst) const {
    for (size_t i = 0; i < Rows; i++) {
      for (size_t j = 0; j < Cols; j++) {
        (*dst)(i, j) = at(i, j) * scalar;
      }
    }
  }
};

template <size_t Size, typename S = double>
class Vector : public Matrix<Size, 1, S> {
 public:
  using Data = std::array<S, Size>;

  constexpr Vector() : Matrix<Size, 1, S>() {}
  constexpr Vector(const Data &vec) {
    for (size_t i = 0; i < Size; i++) {
      Matrix<Size, 1, S>::data_[i][0] = vec[i];
    }
  }

  virtual void Print(std::ostream &os) const override {
    os << '[';
    for (size_t i = 0; i < Size - 1; i++) {
      os << Matrix<Size, 1, S>::at(i, 0) << ", ";
    }
    os << Matrix<Size, 1, S>::at(Size - 1, 0) << ']';
  }

  constexpr S &operator()(const size_t i) {
    return Matrix<Size, 1, S>::data_[i][0];
  }

  constexpr const S &at(const size_t i) const {
    return Matrix<Size, 1, S>::at(i, 0);
  }

  constexpr S Dot(const Vector<Size, S> &vec) const {
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

template <size_t Rows, size_t Cols, typename S = double>
std::ostream &operator<<(std::ostream &os, const Matrix<Rows, Cols, S> &mat) {
  mat.Print(os);
  return os;
}

template <size_t Rows, size_t Cols, typename S = double>
constexpr Matrix<Rows, Cols, S> operator*(const S scalar,
                                          const Matrix<Rows, Cols, S> &mat) {
  return mat * scalar;
}

template <size_t Rows, size_t Cols, typename S = double>
constexpr Vector<Rows, S> operator*(const Matrix<Rows, Cols, S> &mat,
                                    const Vector<Rows, S> &vec) {
  Vector<Rows, S> product;
  Matrix<Rows, Cols, S>::Multiply(mat, vec, &product);
  return product;
}

using Matrix2 = Matrix<2, 2>;
using Matrix3 = Matrix<3, 3>;
using Matrix4 = Matrix<4, 4>;
using Matrix5 = Matrix<5, 5>;
using Matrix6 = Matrix<6, 6>;
using Vector2 = Vector<2>;
using Vector3 = Vector<3>;
using Vector4 = Vector<4>;
using Vector5 = Vector<5>;
using Vector6 = Vector<6>;

}  // namespace spartonautics::linalg

#endif  // MATRIX_H_
