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

#include "tf2/time_cache.h"
#include "tf2/exceptions.h"

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <cassert>

namespace tf2 {


TimeCache::TimeCache(tf2::Duration max_storage_time)
: max_storage_time_(max_storage_time)
{}

namespace cache { // Avoid ODR collisions https://github.com/ros/geometry2/issues/175 
// hoisting these into separate functions causes an ~8% speedup.  Removing calling them altogether adds another ~10%
void createExtrapolationException1(TimePoint t0, TimePoint t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation at time " << displayTimePoint(t0) << ", but only time " << displayTimePoint(t1) << " is in the buffer";
    *error_str = ss.str();
  }
}

void createExtrapolationException2(TimePoint t0, TimePoint t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the future.  Requested time " << displayTimePoint(t0) << " but the latest data is at time " << displayTimePoint(t1);
    *error_str = ss.str();
  }
}

void createExtrapolationException3(TimePoint t0, TimePoint t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the past.  Requested time " << displayTimePoint(t0) << " but the earliest data is at time " << displayTimePoint(t1);
    *error_str = ss.str();
  }
}
} // namespace cache


Maybe<RelTransform> TimeCache::getData(TimePoint time){
  auto i = getInsertionIt(time);
  if (i==storage.begin()) {
    // todo: raise extrapolation error
  }
  else if (i==storage.end())
  {
    // todo : raise extrapolation error
  }

  auto &t0 = i[0];
  auto &t1 = i[1];
  if (t0.second.parent != t1.second.parent)
  {
    // todo: raise error
  }

  RelTransform result;
  tf2Scalar ratio = double((time - t0.first).count()) / double((t1.first - t0.first).count());
  result.translation.setInterpolate3(t0.second.translation, t1.second.translation, ratio);
  result.rotation = slerp(t0.second.rotation, t1.second.rotation, ratio);
  return {result};
}

void TimeCache::insertData(TimePoint time, RelTransform reltf)
{
  auto it = getInsertionIt(time);
  Storage ns;
  storage.insert(it,{time, reltf});
  purge_old();
}

void TimeCache::reset()
{
  storage.clear();
}

unsigned int TimeCache::getListLength()
{
  return (unsigned int)storage.size();
}

TimePoint TimeCache::getLatestTimestamp()
{   
  if (storage.empty()) return TimePoint(); //empty list case
  return storage.front().first;
}

TimePoint TimeCache::getOldestTimestamp()
{   
  if (storage.empty()) return TimePoint(); //empty list case
  return storage.back().first;
}

void TimeCache::purge_old()
{
  if (storage.empty())
    return;

  TimePoint latest_time = storage.back().first;
  storage.erase(storage.begin(),getInsertionIt(latest_time - max_storage_time_));
} // namespace tf2
}
