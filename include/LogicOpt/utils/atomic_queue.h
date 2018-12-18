/**
 * atomic_queue.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_UTILS_ATOMIC_QUEUE_H_
#define LOGIC_OPT_UTILS_ATOMIC_QUEUE_H_

#include <condition_variable>  // std::condition_variable
#include <csignal>             // std::sig_atomic_t
#include <exception>           // std::runtime_error
#include <mutex>               // std::mutex, std::unique_lock
#include <queue>               // std::queue

template <typename T>
class AtomicQueue {

 public:
 
  T Pop() {
    std::unique_lock<std::mutex> lock(m_);
    while (!finish_ && queue_.empty()) {
      cv_.wait(lock);
      if (terminate_) {
        lock.unlock();
        throw std::runtime_error("AtomicQueue::Pop(): Termination requested.");
      }
    }
    if (queue_.empty()) {
      lock.unlock();
      throw std::runtime_error("AtomicQueue::Pop(): Queue marked as finished.");
    }

    T item = queue_.front();
    queue_.pop();
    return item;
  }
 
  void Push(const T& item) {
    std::unique_lock<std::mutex> lock(m_);
    queue_.push(item);
    lock.unlock();
    cv_.notify_one();
  }
 
  void Push(T&& item) {
    std::unique_lock<std::mutex> lock(m_);
    queue_.push(std::move(item));
    lock.unlock();
    cv_.notify_one();
  }

  template<class... Args>
  void Emplace(Args&&... args) {
    std::unique_lock<std::mutex> lock(m_);
    queue_.emplace(args...);
    lock.unlock();
    cv_.notify_one();
  }

  void Finish() {
    finish_ = true;
    cv_.notify_all();
  }

  void Terminate() {
    terminate_ = true;
    cv_.notify_all();
  }
 
 private:

  std::queue<T> queue_;
  std::mutex m_;
  std::condition_variable cv_;
  std::sig_atomic_t terminate_ = false;
  std::sig_atomic_t finish_ = false;

};

#endif  // LOGIC_OPT_UTILS_ATOMIC_QUEUE_H_
