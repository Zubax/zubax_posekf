/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <eigen3/Eigen/Eigen>
#include <cmath>

/**
 * Wolfram Mathematica compatibility helpers.
 * These definitions allow to directly use code produced by the CForm[] function.
 */
namespace mathematica
{
/**
 * Implementation details, do not use directly.
 */
namespace impl
{

template <typename Scalar, int Size>
using RowVector = Eigen::Matrix<Scalar, 1, Size, Eigen::RowMajor>;

template <typename Scalar, int Size, typename Head>
inline void fillVector(RowVector<Scalar, Size>& vector, int next_index, Head head)
{
    vector[next_index] = static_cast<Scalar>(head);
}

template <typename Scalar, int Size, typename Head, typename... Tail>
inline void fillVector(RowVector<Scalar, Size>& vector, int next_index, Head head, Tail... tail)
{
    vector[next_index] = static_cast<Scalar>(head);
    fillVector(vector, next_index + 1, tail...);
}

template <typename Scalar, int Rows, int Columns>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix, int next_row,
                       const impl::RowVector<Scalar, Columns>& head)
{
    matrix.row(next_row) = head;
}

template <typename Scalar, int Rows, int Columns, typename... Tail>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix, int next_row,
                       const impl::RowVector<Scalar, Columns>& head, Tail... tail)
{
    matrix.row(next_row) = head;
    fillMatrix(matrix, next_row + 1, tail...);
}

} // namespace impl

/**
 * Creates a row vector of arbitrary length, represented as a static row vector Eigen::Matrix<>.
 * Scalar type defaults to double.
 */
template <typename Scalar = double, typename... Tail>
inline impl::RowVector<Scalar, sizeof...(Tail)>
List(Tail... tail)
{
    impl::RowVector<Scalar, sizeof...(Tail)> vector;
    impl::fillVector(vector, 0, tail...);
    return vector;
}

/**
 * Creates a static matrix (Eigen::Matrix<>) of arbitrary size from a set of row vectors.
 * The matrix will use default layout, which is columnn-major for Eigen.
 * Note that row vectors of unequal size trigger a compile-time error.
 * This function is totally type safe.
 */
template <typename Scalar, int Columns, typename... Tail>
inline Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns>
List(const impl::RowVector<Scalar, Columns>& head, Tail... tail)
{
    Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns> matrix;
    matrix.setZero();
    impl::fillMatrix(matrix, 0, head, tail...);
    return matrix;
}

/**
 * Mathematica function aliases
 */
template <typename A, typename B> inline decltype(std::pow(A(), B())) Power(A a, B b) { return std::pow(a, b); }

template <typename T> inline decltype(std::sqrt(T())) Sqrt(T x) { return std::sqrt(x); }

template <typename T> inline decltype(std::abs(T())) Abs(T x) { return std::abs(x); }
template <typename T> inline decltype(std::exp(T())) Exp(T x) { return std::exp(x); }
template <typename T> inline decltype(std::log(T())) Log(T x) { return std::log(x); }

template <typename T> inline decltype(std::sin(T())) Sin(T x) { return std::sin(x); }
template <typename T> inline decltype(std::cos(T())) Cos(T x) { return std::cos(x); }
template <typename T> inline decltype(std::tan(T())) Tan(T x) { return std::tan(x); }

template <typename T> inline decltype(std::asin(T())) ArcSin(T x) { return std::asin(x); }
template <typename T> inline decltype(std::acos(T())) ArcCos(T x) { return std::acos(x); }
template <typename T> inline decltype(std::atan(T())) ArcTan(T x) { return std::atan(x); }

/// X,Y swapped!
template <typename X, typename Y> inline decltype(std::atan2(Y(), X())) ArcTan(X x, Y y) { return std::atan2(y, x); }

template <typename T> inline decltype(std::sinh(T())) Sinh(T x) { return std::sinh(x); }
template <typename T> inline decltype(std::cosh(T())) Cosh(T x) { return std::cosh(x); }
template <typename T> inline decltype(std::tanh(T())) Tanh(T x) { return std::tanh(x); }

template <typename Scalar, int Options>
inline Scalar Norm(const Eigen::Quaternion<Scalar, Options>& x) { return x.norm(); }

/**
 * Mathematica constants
 */
static constexpr double E      = 2.71828182845904523536029;
static constexpr double Pi     = 3.14159265358979323846264;
static constexpr double Degree = 0.01745329251994329576924;

}
