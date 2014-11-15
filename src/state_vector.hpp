/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include "linear_algebra.hpp"
#include "mathematica.hpp"

namespace zubax_posekf
{

struct StateVector
{
    static constexpr int Size = <* Length[Flatten[x]] *>;

    Vector<Size> x;

private:
    <* Riffle["Scalar& " <> ToString[#1] <> " = x[" <> ToString[#2] <>
           "];" & @@@ Transpose[{Flatten[x], Range[0, Length[x] - 1]}], "\n    "] // StringJoin
    *>

public:
    StateVector()
    {
        x.setZero();
    }

    static Vector<Size> Qmindiag() const
    {
        <* generateCPPReturnExpressionWithCSE[Transpose@{Qmindiag}, "        "] *>
    }

    static Vector<Size> Pinitdiag() const
    {
        <* generateCPPReturnExpressionWithCSE[Transpose@{Pinitdiag}, "        "] *>
    }

    void normalize()
    {
        <* generateCPPExpressionWithCSE["normx", normx, "        "] *>
        x = normx;
    }

    Vector<Size> f(double dt) const
    {
        <* generateCPPReturnExpressionWithCSE[f, "        "] *>
    }

    Matrix<Size, Size> F(double dt) const
    {
        <* generateCPPReturnExpressionWithCSE[F, "        "] *>
    }

    <* Riffle[generateCPPGetterForSymbol /@
         Select[Names["X`*"], (Symbol[#][[0]] === List || Symbol[#][[0]] == Quaternion) &],
        "\n\n    "] // StringJoin
    *>

    <* Riffle[generateCPPSetterForSymbol /@
         Select[Names["X`*"], (Symbol[#][[0]] === List || Symbol[#][[0]] == Quaternion) &],
        "\n\n    "] // StringJoin
    *>

    <* Riffle[generateCPPGetterForSymbol /@ Names["H`*"], "\n\n    "] // StringJoin
    *>
};

}
