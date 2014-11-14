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

    <* Riffle["Scalar& " <> ToString[#1] <> " = x[" <> ToString[#2] <> "];" &
              @@@ Transpose[{Flatten[x], Range[0, Length[x] - 1]}], "\n    "] // StringJoin
    *>

    StateVector()
    {
        x.setZero();
    }

    Matrix<Size, 1> f(double dt) const
    {
        <* generateCPPReturnExpressionWithCSE[f, "        "] *>
    }

    Matrix<Size, Size> F(double dt) const
    {
        <* generateCPPReturnExpressionWithCSE[F, "        "] *>
    }

    <* Riffle[generateCPPGetterForSymbol /@ Join[Names["Global`H*"], Names["Global`h*"]], "\n\n    "] // StringJoin
    *>
};

}
