/*
 * Copyright (c) 2014 Zubax, zubax.com
 * Please refer to the file LICENSE for terms and conditions.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <map>
#include <typeinfo>

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

    ConstIterator findAfter(double ts) const { return map_.upper_bound(ts); }
    Iterator      findAfter(double ts)       { return map_.upper_bound(ts); }

    ConstIterator findMatchingOrAfter(double ts) const { return map_.lower_bound(ts); }
    Iterator      findMatchingOrAfter(double ts)       { return map_.lower_bound(ts); }

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

    /**
     * Returns the iterator before end().
     * @throws std::range_error if the history is empty.
     */
    ConstIterator getNewestWithCheck() const { return const_cast<HistoryKeeper<T>*>(this)->getNewestWithCheck(); }
    Iterator getNewestWithCheck()
    {
        if (map_.empty())
        {
            throw std::range_error("Can't obtain the newest entry because the history is empty. Item type is: " +
                                   std::string(typeid(T).name()));
        }
        return --std::end(map_);
    }

    /**
     * Same as begin(), with range check.
     * @throws std::range_error if the history is empty.
     */
    ConstIterator getOldestWithCheck() const { return const_cast<HistoryKeeper<T>*>(this)->getOldestWithCheck(); }
    Iterator getOldestWithCheck()
    {
        if (map_.empty())
        {
            throw std::range_error("Can't obtain the oldest entry because the history is empty. Item type is: " +
                                   std::string(typeid(T).name()));
        }
        return std::begin(map_);
    }

    Iterator begin() { return std::begin(map_); }
    Iterator end()   { return std::end(map_); }

    ConstIterator begin() const { return std::begin(map_); }
    ConstIterator end()   const { return std::end(map_); }

    unsigned size() const { return static_cast<unsigned>(map_.size()); }

    bool empty() const { return map_.empty(); }
};

}
