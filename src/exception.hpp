/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <stdexcept>

namespace zubax_posekf
{

class Exception : std::runtime_error
{
public:
    Exception(const std::string& text) : std::runtime_error(text) { }
};

inline void enforce(const char* error_msg, bool condition)
{
    if (!condition)
    {
        throw Exception(error_msg);
    }
}

}
