/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */


#pragma once

#include <map>

namespace zubax_posekf
{
/**
 * Ordered state buffer.
 * Most operations are O(N).
 */
template <typename T>
class HistoryKeeper
{
    // TODO use pool allocator
    typedef std::map<double, T> Map;

    Map map_;

public:
    typedef typename Map::iterator Iterator;
    typedef typename Map::const_iterator ConstIterator;

    void add(double ts, const T& value)
    {
        map_.emplace(ts, value);
    }

    /**
     * Removes the specified position and everything older than that.
     */
    void removeOlderThanInclusive(double ts) { removeOlderThanInclusive(map_.upper_bound(ts)); }
    void removeOlderThanInclusive(ConstIterator it)
    {
        map_.erase(std::begin(map_), it);
    }

    /**
     * Removes the specified position and everything newer than that.
     */
    void removeNewerThanInclusive(double ts) { removeNewerThanInclusive(map_.lower_bound(ts)); }
    void removeNewerThanInclusive(ConstIterator it)
    {
        map_.erase(it, std::end(map_));
    }

    /**
     * Returns an iterator to the closest item which timestamp is strictly lower than the requested timestamp.
     * If the requested timestamp is older than the oldest entry, end() will be returned.
     * If the requested timestamp is newer than the newest entry, (end() - 1) will be returned.
     */
    ConstIterator findBefore(double ts) const { return const_cast<HistoryKeeper<T>*>(this)->findBefore(ts); }
    Iterator findBefore(double ts)
    {
        auto it = map_.lower_bound(ts);         // First greater or equal ts
        if (it == end())                        // All less than ts
        {
            return (size() > 0U) ? --end() : end();
        }
        if (it == begin())                      // All greater than ts
        {
            return end();
        }
        return --it;
    }

    enum class Availability { TooOld, Available, InFuture };

    Availability checkAvailability(double ts) const
    {
        if ((size() <= 0U) || ((--end())->first < ts))
        {
            return Availability::InFuture;
        }
        if (begin()->first > ts)
        {
            return Availability::TooOld;
        }
        return Availability::Available;
    }

    Iterator begin() { return std::begin(map_); }
    Iterator end()   { return std::end(map_); }

    ConstIterator begin() const { return std::begin(map_); }
    ConstIterator end()   const { return std::end(map_); }

    unsigned size() const { static_cast<unsigned>(map_.size()); }
};

}
