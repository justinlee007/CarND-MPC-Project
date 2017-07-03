#ifndef TRACKER_H
#define TRACKER_H

class Tracker {
 private:
  int sample_size_;
  int count_ = 0;

  // CTE tracking
  double best_cte_ = 1.0;
  double worst_cte_ = 0.0;
  double total_cte_ = 0.0;
  double ave_cte_ = 0.0;

  // Throttle tracking
  double ave_throttle_ = 0.0;
  double total_throttle_ = 0.0;

  // Speed tracking
  double best_speed_ = 0.0;
  double ave_speed_ = 0.0;
  double total_speed_ = 0.0;

  // Cost tracking
  double best_cost_ = 0.0;
  double worst_cost_ = 0.0;
  double total_cost_ = 0.0;
  double ave_cost_ = 0.0;

  // Location tracking
  double starting_x_ = 0.0;
  double starting_y_ = 0.0;
  double best_lap_ = 0.0;
  double ave_lap_ = 0.0;
  double total_lap_ = 0.0;
  int num_laps_ = -1;
  bool is_lapping_ = false;

  // Throughput tracking
  time_t init_time_;
  double ave_tps_ = 0.0;

  void printTracking();

 public:

  /**
   * Constructor.
   */
  Tracker();

  /**
   * Destructor.
   */
  virtual ~Tracker();

  void init(int sample_size);

  double getAveTps();

  void onMessageProcessed(double cte, double speed, double throttle, double cost, double x, double y);

};

#endif //TRACKER_H
