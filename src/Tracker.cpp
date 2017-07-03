#include <cmath>
#include <iostream>
#include "Tracker.h"

using namespace std::chrono;

Tracker::Tracker() {}
Tracker::~Tracker() {}

auto lap_timer_ = steady_clock::now();

void Tracker::init(int sample_size) {
  sample_size_ = sample_size;
}

double Tracker::getAveTps() {
  return count_ / difftime(time(NULL), init_time_);;
}

void Tracker::onMessageProcessed(double cte, double speed, double throttle, double cost, double x, double y) {
  best_cte_ = fmin(fabs(cte), best_cte_);
  worst_cte_ = fmax(fabs(cte), worst_cte_);
  total_cte_ += fabs(cte);

  best_speed_ = fmax(speed, best_speed_);
  total_speed_ += speed;

  total_throttle_ += throttle;

  best_cost_ = fmin(cost, best_cost_);
  worst_cost_ = fmax(cost, worst_cost_);
  total_cost_ += cost;

  if (count_ == 0) {
    time(&init_time_);
    starting_x_ = x;
    starting_y_ = y;
  } else {
    if (x == starting_x_ && y == starting_y_) {
      is_lapping_ = true;
    } else if (is_lapping_) {
      if (++num_laps_ > 0) {
        auto current_lap = duration_cast<milliseconds>(steady_clock::now() - lap_timer_).count();
        best_lap_ = (num_laps_ == 1) ? current_lap : fmin(current_lap, best_lap_);
        total_lap_ += current_lap;
        ave_lap_ = total_lap_ / num_laps_;
        printf("best_lap_=%.2fs, ave_lap_=%.2fs, num_laps_=%i\n", (best_lap_ / 1000), (ave_lap_ / 1000), num_laps_);
      }
      lap_timer_ = steady_clock::now();
      is_lapping_ = false;
    }
  }
  if ((++count_ % sample_size_) == 0) {
    printTracking();
  }
}

void Tracker::printTracking() {
  ave_cte_ = total_cte_ / count_;
  ave_speed_ = total_speed_ / count_;
  ave_throttle_ = total_throttle_ / count_;
  ave_cost_ = total_cost_ / count_;
  ave_tps_ = count_ / difftime(time(NULL), init_time_);
  printf("best_cte_=%.5f, worst_cte_=%.5f, ave_cte_=%.5f\n", best_cte_, worst_cte_, ave_cte_);
  printf("best_cost_=%.0f, worst_cost_=%.0f, ave_cost_=%.0f\n", best_cost_, worst_cost_, ave_cost_);
  printf("best_speed_=%.2f, ave_speed_=%.2f, ave_throttle_=%.3f, ave_tps_=%.1f\n", best_speed_, ave_speed_, ave_throttle_, ave_tps_);
}
