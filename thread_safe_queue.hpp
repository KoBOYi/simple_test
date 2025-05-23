#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
template <typename T>
class ThreadSafeQueue
{
    public:

    void push(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
  }

  bool pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty()) {
        queue_.pop();
        return true;
    }
    return false;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;

}

#endif // THREAD_SAFE_QUEUE_HPP