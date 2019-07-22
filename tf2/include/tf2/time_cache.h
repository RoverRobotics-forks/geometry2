/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include "buffer_core.h"
#include "exceptions.h"
#include "transform_storage.h"

#include <list>
#include <memory>
#include <sstream>

#include <deque>

#include "tf2/time_cache_interface.h"
#include "tf2/visibility_control.h"

namespace tf2
{

constexpr Duration TIMECACHE_DEFAULT_MAX_STORAGE_TIME = std::chrono::seconds(10); //!< default value of 10 seconds storage

/// Represents the transform history of a given frame, whose value may change
/// smoothly over time. Allows inserting new data, interpolating between
/// existing data, and purging old data.
class TimeCache : public TimeCacheInterface {
  /// Cache entry data type.
  /// Each cache entry has both a time and the frame's relation to its parent
  using Storage = std::pair<TimePoint, RelTransform>;

  /// All cache entries. Insertions will happen mostly at or near one end and
  /// purges at the other, so a deque gives good performance
  std::deque<Storage> storage;

  decltype(TimeCache::storage)::iterator getInsertionIt(TimePoint time){
    // binary search for the location where
    auto result = std::upper_bound(storage.begin(),storage.end(), time,
        [](TimePoint t, Storage t2) -> bool {return t < t2.first; });

    assert (storage.empty() ||
        result == storage.end() && storage.back().time < time ||
        result == storage.begin() && time <= result -> time ||
        (result-1)->time < time && time <= result->time
    );
    return result;
  }
  /// Time horizon for purging old cache entries
  tf2::Duration max_storage_time_;

  /// Purge old cache entries, before the time horizon
  void purge_old();

public:
  TF2_PUBLIC
  TimeCache(tf2::Duration max_storage_time = TIMECACHE_DEFAULT_MAX_STORAGE_TIME);

  /// Virtual methods
  TF2_PUBLIC
  Maybe<RelTransform> getData(TimePoint time) override;
  TF2_PUBLIC
  void insertData(TimePoint,RelTransform) override;
  TF2_PUBLIC
  virtual void reset();
};

/// Represents the rigid transform of a given frame, whose relative transform
/// does not change with time.
/// Inserting new data completely forgets the old value.
class StaticCache : public TimeCacheInterface {
  RelTransform storage;
public:
  /// Virtual methods
  TF2_PUBLIC
  Maybe<RelTransform> getData(TimePoint time) override;
  TF2_PUBLIC
  void insertData(TimePoint time, RelTransform data) override;
  TF2_PUBLIC
  void reset() override;
  /// Debugging information methods
  TF2_PUBLIC
  virtual unsigned int getListLength();
  TF2_PUBLIC
  virtual TimePoint getLatestTimestamp();
  TF2_PUBLIC
  virtual TimePoint getOldestTimestamp();
};
}

#endif // TF2_TIME_CACHE_H
