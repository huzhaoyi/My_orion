/* 线程安全 FIFO 任务队列，供 Worker 消费 */

#ifndef ORION_MTC_ORCHESTRATION_TASK_QUEUE_HPP
#define ORION_MTC_ORCHESTRATION_TASK_QUEUE_HPP

#include "orion_mtc/core/manipulation_job.hpp"
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

namespace orion_mtc
{

class TaskQueue
{
public:
  /** 按 priority 插入（数值越大越靠前），同优先级 FIFO */
  void push(const ManipulationJob& job);
  bool tryPop(ManipulationJob& job);
  bool waitPop(ManipulationJob& job, std::chrono::milliseconds timeout);
  bool empty() const;
  std::size_t size() const;
  void clear();
  /** 窥视队首，不弹出；队列空返回 false */
  bool peekFront(ManipulationJob& job) const;
  /** 按 job_id 移除队列中第一个匹配项；找到并移除返回 true，可选通过 out_removed 返回被移除的 job */
  bool removeById(const std::string& job_id, ManipulationJob* out_removed = nullptr);

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ManipulationJob> queue_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_ORCHESTRATION_TASK_QUEUE_HPP
