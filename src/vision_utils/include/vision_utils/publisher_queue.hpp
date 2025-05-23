#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

template <typename MsgT, typename PubT> class PublisherQueue {
public:
  PublisherQueue(PubT publisher, size_t max_queue_size = 2)
      : publisher_(publisher), max_queue_size_(max_queue_size), running_(true) {
    thread_ = std::thread(&PublisherQueue::run, this);
  }

  ~PublisherQueue() { stop(); }

  void enqueue(typename MsgT::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= max_queue_size_) {
        queue_.pop(); // Drop oldest
      }
      queue_.push(msg);
    }
    cv_.notify_one();
  }

  void stop() {
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void run() {
    while (running_) {
      typename MsgT::SharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return !queue_.empty() || !running_; });
        if (!running_ && queue_.empty())
          break;
        if (!queue_.empty()) {
          msg = queue_.front();
          queue_.pop();
        }
      }
      if (msg) {
        publisher_.publish(msg);
      }
    }
  }

  PubT publisher_;
  size_t max_queue_size_;
  std::queue<typename MsgT::SharedPtr> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  std::atomic<bool> running_;
};
