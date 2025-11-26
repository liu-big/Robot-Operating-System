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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"和任何明示或暗示的保证，包括但不限于适销性和特定用途适用性的暗示保证。
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

/**
 * @file cache.cpp
 * @brief TF库的时间缓存实现文件。
 * 该文件包含了用于存储和管理时间戳转换的TimeCache类，
 * 以及相关的辅助函数，用于处理时间查找、插值和异常情况。
 */

#include "tf/time_cache.h"
#include "tf/exceptions.h"

#include "tf/LinearMath/Transform.h"
#include <geometry_msgs/TransformStamped.h>

#include "ros/assert.h"

using namespace tf;

/**
 * @brief TransformStorage类的默认构造函数。
 * @details 初始化TransformStorage对象。
 */
TransformStorage::TransformStorage()
{
}

/**
 * @brief TransformStorage类的构造函数。
 * @param data 包含转换数据（旋转、平移、时间戳）的StampedTransform对象。
 * @param frame_id 帧的紧凑ID。
 * @param child_frame_id 子帧的紧凑ID。
 * @details 使用给定的StampedTransform数据、父帧ID和子帧ID初始化TransformStorage对象。
 */
TransformStorage::TransformStorage(const StampedTransform& data, CompactFrameID frame_id,
                                   CompactFrameID child_frame_id)
: rotation_(data.getRotation())
, translation_(data.getOrigin())
, stamp_(data.stamp_)
, frame_id_(frame_id)
, child_frame_id_(child_frame_id)
{ }

/**
 * @brief TimeCache类的构造函数。
 * @param max_storage_time 缓存中允许存储的最大时间长度。
 * @details 初始化TimeCache对象，设置最大存储时间。
 */
TimeCache::TimeCache(ros::Duration max_storage_time)
: max_storage_time_(max_storage_time)
{}

// hoisting these into separate functions causes an ~8% speedup.  Removing calling them altogether adds another ~10%
/**
 * @brief 创建一个空的异常信息。
 * @param error_str 指向存储错误信息的字符串指针。
 * @details 当缓存为空时，生成相应的错误信息。
 */
void createEmptyException(std::string *error_str)
{
  if (error_str)
  {
    *error_str = "Unable to lookup transform, cache is empty";
  }
}

/**
 * @brief 创建一个外推异常信息（时间点t0在缓存中时间点t1之前）。
 * @param t0 请求的时间点。
 * @param t1 缓存中最近的时间点。
 * @param error_str 指向存储错误信息的字符串指针。
 * @details 当请求的时间早于缓存中的最早数据时，生成外推错误信息。
 */
void createExtrapolationException1(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation at time " << t0 << ", but only time " << t1 << " is in the buffer";
    *error_str = ss.str();
  }
}

/**
 * @brief 创建一个外推异常信息（时间点t0在缓存中时间点t1之后）。
 * @param t0 请求的时间点。
 * @param t1 缓存中最新的时间点。
 * @param error_str 指向存储错误信息的字符串指针。
 * @details 当请求的时间晚于缓存中的最新数据时，生成外推错误信息。
 */
void createExtrapolationException2(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the future.  Requested time " << t0 << " but the latest data is at time " << t1;
    *error_str = ss.str();
  }
}

/**
 * @brief 创建一个外推异常信息（时间点t0在缓存中时间点t1之前）。
 * @param t0 请求的时间点。
 * @param t1 缓存中最旧的时间点。
 * @param error_str 指向存储错误信息的字符串指针。
 * @details 当请求的时间早于缓存中的最早数据时，生成外推错误信息。
 */
void createExtrapolationException3(ros::Time t0, ros::Time t1, std::string* error_str)
{
  if (error_str)
  {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the past.  Requested time " << t0 << " but the earliest data is at time " << t1;
    *error_str = ss.str();
  }
}

/**
 * @brief 在缓存中查找最接近目标时间的两个转换。
 * @param one 指向第一个（较旧）转换的指针。
 * @param two 指向第二个（较新）转换的指针。
 * @param target_time 目标时间。
 * @param error_str 指向存储错误信息的字符串指针。
 * @return 找到的转换数量（0、1或2）。
 * @details 该函数用于在缓存中查找给定时间点前后最近的两个变换数据，以便进行插值。
 */
uint8_t TimeCache::findClosest(const TransformStorage*& one, const TransformStorage*& two, ros::Time target_time, std::string* error_str)
{
  //No values stored
  if (storage_.empty())
  {
    createEmptyException(error_str);
    return 0;
  }

  //If time == 0 return the latest
  if (target_time.isZero())
  {
    one = &(*storage_.rbegin());
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    const TransformStorage& ts = *storage_.begin();
    if (ts.stamp_ == target_time)
    {
      one = &ts;
      return 1;
    }
    else
    {
      createExtrapolationException1(target_time, ts.stamp_, error_str);
      return 0;
    }
  }

  ros::Time latest_time = (*storage_.rbegin()).stamp_;
  ros::Time earliest_time = (*(storage_.begin())).stamp_;

  if (target_time == latest_time)
  {
    one = &(*storage_.rbegin());
    return 1;
  }
  else if (target_time == earliest_time)
  {
    one = &(*storage_.begin());
    return 1;
  }
  // Catch cases that would require extrapolation
  else if (target_time > latest_time)
  {
    createExtrapolationException2(target_time, latest_time, error_str);
    return 0;
  }
  else if (target_time < earliest_time)
  {
    createExtrapolationException3(target_time, earliest_time, error_str);
    return 0;
  }

  //Create a temporary object to compare to when searching the lower bound via std::set
  TransformStorage tmp;
  tmp.stamp_ = target_time;

  //Find the first value equal or higher than the target value
  L_TransformStorage::iterator storage_it = storage_.upper_bound(tmp);

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  two = &*(storage_it); //Newer
  one = &*(--storage_it); //Older

  return 2;

}

/**
 * @brief 对两个转换之间的数据进行插值。
 * @param one 第一个（较旧）转换。
 * @param two 第二个（较新）转换。
 * @param time 目标时间。
 * @param output 存储插值结果的TransformStorage对象。
 * @details 根据给定的时间，对两个变换数据进行线性插值，得到在该时间点的变换。
 */
void TimeCache::interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output)
{
  // Check for zero distance case
  if( two.stamp_ == one.stamp_ )
  {
    output = two;
    return;
  }
  //Calculate the ratio
  tfScalar ratio = (time.toSec() - one.stamp_.toSec()) / (two.stamp_.toSec() - one.stamp_.toSec());

  //Interpolate translation
  output.translation_.setInterpolate3(one.translation_, two.translation_, ratio);

  //Interpolate rotation
  output.rotation_ = slerp( one.rotation_, two.rotation_, ratio);

  output.stamp_ = one.stamp_;
  output.frame_id_ = one.frame_id_;
  output.child_frame_id_ = one.child_frame_id_;
}

/**
 * @brief 从缓存中获取指定时间的数据。
 * @param time 请求的时间。
 * @param data_out 存储获取到的数据的TransformStorage对象。
 * @param error_str 指向存储错误信息的字符串指针。
 * @return 如果数据可用则返回true，否则返回false。
 * @details 尝试从缓存中获取指定时间点的变换数据，如果需要会进行插值。
 */
bool TimeCache::getData(ros::Time time, TransformStorage & data_out, std::string* error_str) //returns false if data not available
{
  const TransformStorage* p_temp_1 = NULL;
  const TransformStorage* p_temp_2 = NULL;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0)
  {
    return false;
  }
  else if (num_nodes == 1)
  {
    data_out = *p_temp_1;
  }
  else if (num_nodes == 2)
  {
    if( p_temp_1->frame_id_ == p_temp_2->frame_id_){
      interpolate(*p_temp_1, *p_temp_2, time, data_out);
    }
    else
    {
      data_out = *p_temp_1;
    }
  }
  else
  {
    ROS_BREAK();
  }

  return true;
}

/**
 * @brief 获取指定时间点的父帧ID。
 * @param time 请求的时间。
 * @param error_str 指向存储错误信息的字符串指针。
 * @return 父帧的CompactFrameID。
 * @details 查找指定时间点最近的变换数据，并返回其父帧ID。
 */
CompactFrameID TimeCache::getParent(ros::Time time, std::string* error_str)
{
  const TransformStorage* p_temp_1 = NULL;
  const TransformStorage* p_temp_2 = NULL;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0)
  {
    return 0;
  }

  return p_temp_1->frame_id_;
}

/**
 * @brief 插入新的变换数据到缓存中。
 * @param new_data 要插入的TransformStorage对象。
 * @return 如果成功插入则返回true，否则返回false。
 * @details 将新的变换数据按时间顺序插入到缓存中，并处理旧数据的修剪。
 */
bool TimeCache::insertData(const TransformStorage& new_data)
{

  if (storage_.begin() != storage_.end())
  {
      // trying to add data that dates back longer than we want to keep history
      if (storage_.rbegin()->stamp_ > new_data.stamp_ + max_storage_time_)
        return false;

      // if we already have data at that exact time, delete it to ensure the latest data is stored
      if (storage_.rbegin()->stamp_ >= new_data.stamp_)
      {
         L_TransformStorage::iterator storage_it  = storage_.find(new_data);
         if (storage_it != storage_.end())
                storage_.erase(storage_it);
      }
  }

  storage_.insert(storage_.end(), new_data);

  pruneList();

  return true;
}

/**
 * @brief 清空缓存中的所有数据。
 * @details 移除所有存储的变换数据。
 */
void TimeCache::clearList()
{
  storage_.clear();
}

/**
 * @brief 获取缓存中存储的变换数量。
 * @return 缓存中变换的数量。
 * @details 返回当前缓存中包含的变换数据条目数。
 */
unsigned int TimeCache::getListLength()
{
  return storage_.size();
}

/**
 * @brief 获取最新变换的时间戳和父帧ID。
 * @return 包含最新时间戳和父帧ID的pair。
 * @details 返回缓存中最新变换的时间戳及其父帧ID。
 */
P_TimeAndFrameID TimeCache::getLatestTimeAndParent()
{
  if (storage_.empty())
  {
    return std::make_pair(ros::Time(), 0);
  }

  const TransformStorage& ts = *storage_.rbegin();
  return std::make_pair(ts.stamp_, ts.frame_id_);
}

/**
 * @brief 获取缓存中最新变换的时间戳。
 * @return 最新变换的时间戳。
 * @details 返回缓存中最新变换的时间戳，如果缓存为空则返回ros::Time()。
 */
ros::Time TimeCache::getLatestTimestamp()
{
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.rbegin()->stamp_;
}

/**
 * @brief 获取缓存中最旧变换的时间戳。
 * @return 最旧变换的时间戳。
 * @details 返回缓存中最旧变换的时间戳，如果缓存为空则返回ros::Time()。
 */
ros::Time TimeCache::getOldestTimestamp()
{
  if (storage_.empty()) return ros::Time(); //empty list case
  return storage_.begin()->stamp_;
}

/**
 * @brief 修剪缓存中的旧数据。
 * @details 移除超出最大存储时间限制的旧变换数据，保持缓存大小在合理范围内。
 */
void TimeCache::pruneList()
{
  ros::Time latest_time = storage_.rbegin()->stamp_;

  while(!storage_.empty() && storage_.begin()->stamp_ + max_storage_time_ < latest_time)
  {
    storage_.erase(storage_.begin());
  }

}
