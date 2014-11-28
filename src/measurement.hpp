/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
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
