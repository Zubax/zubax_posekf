/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cstdint>
#include <ctime>
#include <string>
#include <stdexcept>

namespace zubax_posekf
{
/**
 * Basic stopwatch, useful for profiling.
 * The first time interval starts automatically at the moment of its creation.
 */
class Stopwatch
{
public:
    enum class ClockType
    {
        SystemRealTime  = CLOCK_REALTIME,
        SystemMonotonic = CLOCK_MONOTONIC,
        Process         = CLOCK_PROCESS_CPUTIME_ID,
        Thread          = CLOCK_THREAD_CPUTIME_ID
    };

private:
    const ClockType clock_type_;
    std::uint64_t ts_usec_ = 0;

public:
    std::uint64_t getClockUSec() const
    {
        auto ts = ::timespec();
        if (clock_gettime(static_cast<int>(clock_type_), &ts) != 0)
        {
            throw std::runtime_error("Failed to get the clock value; clock type: " +
                                     std::to_string(static_cast<int>(clock_type_)));
        }
        return std::uint64_t(std::int64_t(ts.tv_sec) * 1000000L + ts.tv_nsec / 1000L);
    }

    Stopwatch(ClockType arg_clock_type = ClockType::Thread)
        : clock_type_(arg_clock_type)
    {
        ts_usec_ = getClockUSec();
    }

    std::uint64_t restartAndGetElapsedUSec()
    {
        const auto ts = getClockUSec();
        const auto retval = ts - ts_usec_;
        ts_usec_ = ts;
        return retval;
    }

    std::uint64_t getElapsedUSec() const
    {
        return getClockUSec() - ts_usec_;
    }
};

/**
 * Utility class - prints the time duration between its creation and destruction.
 */
class StopwatchPrinter
{
    const char* const name_;
    Stopwatch sw_;

    StopwatchPrinter(const StopwatchPrinter&) = delete;
    StopwatchPrinter& operator=(const StopwatchPrinter&) = delete;

public:
    StopwatchPrinter(const char* name, Stopwatch::ClockType clock_type = Stopwatch::ClockType::Thread)
        : name_(name)
        , sw_(clock_type)
    { }

    ~StopwatchPrinter()
    {
        const float msec = static_cast<float>(sw_.getElapsedUSec()) / 1e3F;
        std::cout << "Stopwatch [" << name_ << "]: " << msec << " ms" << std::endl;
    }
};

}
