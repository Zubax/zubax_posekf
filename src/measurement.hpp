/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

namespace zubax_posekf
{
/**
 * All filter inputs shall inherit this.
 */
struct Measurement
{
    ros::Time timestamp;

    virtual ~Measurement() { }

    bool isValid() const { return timestamp.isValid(); }
};

}
