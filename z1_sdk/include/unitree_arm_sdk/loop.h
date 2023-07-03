#pragma once

#include <chrono>
#include <thread>
#include <functional>
#include <iostream>
#include <memory>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <string.h>

/**
 * @brief A timer to ensure the frequency of thread execution.
 */
class Timer
{
public:
  /**
   * @brief Construct a new Timer object
   * 
   * @param period_sec time, Unit: second
   */
  Timer(double period_sec) {
    period_ = period_sec;
    start(); // default
  }

  double period() {return period_; }

  /**
   * @brief Update the beginning time.
   */
  void start() { start_time_ = std::chrono::steady_clock::now();  }

  /**
   * @brief Caculate the time from the beginning to the present.
   * 
   * @return second
   */
  double elasped_time() 
  {
    auto end_time_ = std::chrono::steady_clock::now();
    auto elasped_time = end_time_ - start_time_;
    size_t t = std::chrono::duration_cast<std::chrono::microseconds>(elasped_time).count();
    return (double)t/1000000.0;
  }

  /**
   * @brief Caculate the remaining time to a period
   * 
   * @return second
   */
  double wait_time() { return period_ - elasped_time(); }

  /**
   * @brief Sleep for wait_time() until a period finished.
   * 
   * If it has timeout, do nothing.
   */
  void sleep()
  {
    double waitTime = wait_time();
    if(waitTime > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(size_t(waitTime*1000000)));
    }
    start();// If the last one ends and then start a new timer.
  }

private:
  double period_;
  std::chrono::steady_clock::time_point start_time_;
};
typedef std::shared_ptr<Timer> TimerPtr;


/**
 * @brief Maintains a thread to run once every period.
 */
class Loop
{
public:
  /**
   * @brief Construct a new Loop object
   * 
   * @param name Indicate what the loop aims to
   * @param period time, Unit: second
   * @param callback the running function pointer
   */
  Loop(std::string name, double period, std::function<void ()> callback) 
    :name_(name), cb_(callback) {
    timer_ = std::make_shared<Timer>(period);
  }
  ~Loop() { shutdown(); }

  void start()
  {
    if(isrunning_)
    {
      if(show_info_) std::cout << "[WARNING] Loop "<< name_ << " is already runnig." << std::endl;
      return;
    }

    isrunning_ = true;
    thread_ = std::thread(&Loop::running_impl, this);
  }

  void shutdown()
  {
    if(!isrunning_) return;
    isrunning_ = false;
    thread_.join();
    if(show_info_) {
      std::cout << "[REPORT] The time out rate of thread " << name_ << " is " 
                << 100.0*(double)timeout_times_/(double)run_times_ << "%" << std::endl;
    }
  }

  void spinOnce()
  {
    timer_->start();
    ++run_times_;
    cb_();

    if(timer_->wait_time() > 0) {
      timer_->sleep();
    } else {
      ++timeout_times_;
      if(show_info_) std::cout << "[Report] Loop "<< name_ 
                              << " timeout. It has cost "<< timer_->elasped_time() << "s\n";
    }
  }

  void showInfo(bool state) { show_info_ = state; } ;
private:
  void running_impl() {
    while (isrunning_) {
      spinOnce();
    }
    if(show_info_) std::cout << "[Loop End] named: "<< name_ << std::endl;
  }

  std::string name_{};
  bool isrunning_ = false;
  std::shared_ptr<Timer> timer_;

  std::function<void ()> cb_;
  std::thread thread_;

  size_t run_times_ = 0;
  size_t timeout_times_ = 0;
  bool show_info_ = false;
};
typedef std::shared_ptr<Loop> LoopPtr;