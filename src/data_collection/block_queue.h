#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

template <class T>
class blockqueue {
 public:
  int put(T&& t) {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.emplace_back(std::move(t));
    cv_.notify_one();
    return static_cast<int>(que_.size());
  }

  int put(const T& t) {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.push_back(t);
    cv_.notify_one();
    return static_cast<int>(que_.size());
  }

  int put_front(T&& t) {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.emplace_front(std::move(t));
    cv_.notify_one();
    return static_cast<int>(que_.size());
  }

  int put_front(const T& t) {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.push_front(t);
    cv_.notify_one();
    return static_cast<int>(que_.size());
  }

  bool get(T& t, uint32_t timeout_ms = 300) {
    std::unique_lock<std::mutex> lck(mtx_);
    if (!que_.empty() || cv_.wait_for(lck, std::chrono::milliseconds(timeout_ms),
                                       [this] { return !que_.empty(); })) {
      t = que_.front();
      que_.pop_front();
      return true;
    }
    return false;
  }

  void pop_front() {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.pop_front();
  }

  void clear() {
    std::lock_guard<std::mutex> lck(mtx_);
    que_.clear();
  }

  size_t size() {
    std::lock_guard<std::mutex> lck(mtx_);
    return que_.size();
  }

 private:
  std::condition_variable cv_;
  std::mutex mtx_;
  std::deque<T> que_;
};
