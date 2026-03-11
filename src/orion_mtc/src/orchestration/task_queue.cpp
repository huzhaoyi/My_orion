#include "orion_mtc/orchestration/task_queue.hpp"
#include <algorithm>

namespace orion_mtc
{

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

bool TaskQueue::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.empty();
}

std::size_t TaskQueue::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

void TaskQueue::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

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
