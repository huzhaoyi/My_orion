/* task_queue：优先级插入、阻塞 pop、clear 与 size */

#include "orion_mtc/orchestration/task_queue.hpp"
#include <algorithm>

namespace orion_mtc
{

/*
 * 大 priority 值靠前插入；唤醒阻塞在 waitPop 上的消费者。
 */
void TaskQueue::push(const ManipulationJob& job)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = queue_.begin();
  for (; it != queue_.end(); ++it)
  {
    if (it->priority < job.priority)
    {
      queue_.insert(it, job);
      cv_.notify_one();
      return;
    }
  }
  queue_.push_back(job);
  cv_.notify_one();
}

/*
 * 非阻塞：空则 false；否则弹出队首 move 到 job。
 */
bool TaskQueue::tryPop(ManipulationJob& job)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty())
  {
    return false;
  }
  job = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

/*
 * 超时内等待非空，成功后与 tryPop 相同弹出语义。
 */
bool TaskQueue::waitPop(ManipulationJob& job, std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, timeout, [this]() { return !queue_.empty(); }))
  {
    return false;
  }
  job = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

/*
 * 线程安全查询队列是否为空。
 */
bool TaskQueue::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.empty();
}

/*
 * 当前排队任务条数。
 */
std::size_t TaskQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

/*
 * 清空队列（不通知 CV；适合停机或丢弃积压）。
 */
void TaskQueue::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

/*
 * 复制队首到 job 但不弹出；空队列返回 false。
 */
bool TaskQueue::peekFront(ManipulationJob& job) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty())
  {
    return false;
  }
  job = queue_.front();
  return true;
}

/*
 * 按 job_id 线性查找并擦除；out_removed 非空时写出被移除项。未找到返回 false。
 */
bool TaskQueue::removeById(const std::string& job_id, ManipulationJob* out_removed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = queue_.begin(); it != queue_.end(); ++it)
  {
    if (it->job_id == job_id)
    {
      if (out_removed)
      {
        *out_removed = *it;
      }
      queue_.erase(it);
      return true;
    }
  }
  return false;
}

}  // namespace orion_mtc
