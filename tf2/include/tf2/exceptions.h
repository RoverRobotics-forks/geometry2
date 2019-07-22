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

#ifndef TF2_EXCEPTIONS_H
#define TF2_EXCEPTIONS_H

#include <stdexcept>
#include <cstdint>

#include <tf2/visibility_control.h>
#include <sstream>
#include <vector>
#include <stdexcept>
namespace tf2{

enum class TF2Error : std::uint8_t {
  NO_ERROR = 0,
  LOOKUP_ERROR = 1,
  CONNECTIVITY_ERROR = 2,
  EXTRAPOLATION_ERROR = 3,
  INVALID_ARGUMENT_ERROR = 4,
  TIMEOUT_ERROR = 5,
  TRANSFORM_ERROR = 6,
  LOOP_ERROR = 7,
  NO_PARENT = 8,
};

/** \brief A base class for all tf2 exceptions 
 * This inherits from ros::exception 
 * which inherits from std::runtime_exception
 */
class TransformException: public std::runtime_error
{ 
public:
  TF2_PUBLIC
  explicit TransformException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };

  TF2_PUBLIC
  virtual TF2Error to_enum() {
     return TF2Error ::TRANSFORM_ERROR;
  };
};

template <typename T>
class Maybe final {
  bool is_result;
  union {
    const T result;
    const std::shared_ptr<TransformException> exception;
  } content;

  Maybe(bool is_result, decltype(content) content): is_result(is_result),content(content){}

public:
  auto operator *(){
    return get();
  }
  auto get() {
    if (!is_result)
      return content.result;
    else
      throw *(content.exception);
  }
  auto is_success() {return !bool(is_result);}
  std::reference_wrapper<TransformException> exc_ref(){
    assert (exception);
    return content.exception.get();
  };
  std::shared_ptr<TransformException> get_exception(){return exception;}

  template<typename... Args>
  static Maybe success(Args&&... args) {
    return Maybe(true, std::forward<Args>(args)...);
  }

  template<typename Exc, typename... Args>
  static Maybe failure(Args&&... args) {
    return Maybe(false, std::make_shared<Exc>(std::forward<Args>(args)...));
  }

  template<typename E, typename... Args>
  explicit Maybe (Args&&... args):result(nullptr), exception(new E(std::forward<Args>(args)...)){};
};



  /** \brief An exception class to notify of no connection
   * 
   * This is an exception class to be thrown in the case 
   * that the Reference Frame tree is not connected between
   * the frames requested. */
class ConnectivityException: public TransformException
{
public:
  static std::string make_error_string (
    std::string &target_frame,
    std::vector<std::string> &target_frame_parents,
    std::string &source_frame,
    std::vector<std::string> &source_frame_parents){
    std::stringstream ss;
    ss << "Could not find a connection between '"<<target_frame<<"' and '"<<
          source_frame<<"' because they are not part of the same tree. ";

    ss << "Path up from target frame: '" << target_frame << "'";
    for (auto &frame: target_frame_parents)
      ss << ", '"<<frame<<"'";

    ss << "; Path up from source frame: '" << source_frame << "'";
    for (auto &frame : source_frame_parents)
      ss << ", '"<<frame<<"'";
  }

  TF2_PUBLIC
  ConnectivityException(
    std::string &target_frame,
    std::vector<std::string> &target_frame_parents,
    std::string &source_frame,
    std::vector<std::string> &source_frame_parents)
  :  TransformException(
    make_error_string(target_frame,target_frame_parents,source_frame,source_frame_parents)
    ) {}

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::CONNECTIVITY_ERROR;
  };
};


/** \brief An exception class to notify of bad frame number 
 * 
 * This is an exception class to be thrown in the case that 
 * a frame not in the graph has been attempted to be accessed.
 * The most common reason for this is that the frame is not
 * being published, or a parent frame was not set correctly 
 * causing the tree to be broken.  
 */
class LookupException: public TransformException
{ 
public:
  TF2_PUBLIC
  explicit LookupException(const std::string frame) : tf2::TransformException("Frame does not exist: " + frame) {  };

  TF2_PUBLIC
  LookupException(const std::string frame, const std::string frame2) : tf2::TransformException("Frames do not exist: "+frame + ", "+ frame) { };
};

  /** \brief An exception class to notify that the requested value would have required extrapolation beyond current limits.
   * 
   */
class ExtrapolationException: public TransformException
{ 
public:
  TF2_PUBLIC
  explicit ExtrapolationException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::EXTRAPOLATION_ERROR;
  };
};

/** \brief An exception class to notify that one of the arguments is invalid
 * 
 * usually it's an uninitalized Quaternion (0,0,0,0)
 * 
 */
class InvalidArgumentException: public TransformException
{ 
public:
  TF2_PUBLIC
  explicit InvalidArgumentException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::INVALID_ARGUMENT_ERROR;
  };
};

class NoParentException: public TransformException
{
  std::string describe(std::string child){
    return "The frame "+child+" does not have a parent";
  }
public:
  TF2_PUBLIC
  explicit NoParentException(const std::string child) : tf2::TransformException(describe(child)) {  };

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::NO_PARENT;
  };
};

/** \brief An exception class to notify that a timeout has occurred
 * 
 * 
 */
class TimeoutException: public TransformException
{ 
public:
  TF2_PUBLIC
  explicit TimeoutException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::TIMEOUT_ERROR;
  };
};

class LoopException: public TransformException {
  static std::string describe (std::vector<std::string> &nodes) {
    std::stringstream ss;
    ss << "Transform tree contains a loop: ";
    bool first = true;
    for (auto & n: nodes) {
      if (first){
        first = false;
      }
      else {
        ss << ", ";
      }
      ss << "'" << n << "'";
    }
  }
public:
  TF2_PUBLIC
  explicit LoopException(std::vector<std::string> &nodes): tf2::TransformException(describe(nodes)){};

  TF2_PUBLIC
  TF2Error to_enum() override {
    return TF2Error::LOOP_ERROR;
  };

};


}


#endif //TF2_EXCEPTIONS_H
