#pragma once

#include "exceptions.h"
#include "time.h"
#include "transform_storage.h"
#include "visibility_control.h"
namespace tf2 {

class TimeCacheInterface {

public:
  using Ptr = std::shared_ptr<TimeCacheInterface>;
  using Ref = std::reference_wrapper<TimeCacheInterface>;

  TF2_PUBLIC
  virtual Maybe<RelTransform> getData(TimePoint) = 0;
  TF2_PUBLIC
  virtual void insertData(TimePoint, RelTransform) = 0;
  TF2_PUBLIC
  virtual void reset() = 0;
};
}
