#pragma once

#include <memory>
#include <vector>

#include "agilib/base/module.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/imu_sample.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/types/ball_state.hpp"
#include "agilib/types/point_cloud.hpp"

namespace agi {

  struct Frame {
    Scalar        t;         // the timestamp
    std::optional<Pose> pose; // present if we got a pose, empty if it’s a “blank” frame
  };

class EstimatorBaseBall : public Module<EstimatorBaseBall> {
 public:
  EstimatorBaseBall(const std::string& name = "Estimator");
  virtual ~EstimatorBaseBall();

  /// Initialize the filter.
  virtual bool initialize(const BallState& state) = 0;

  /// Add measurement
  // virtual bool addPose(const Pose& pose) = 0;
  virtual bool addPointCloud(const PointCloud& point_cloud) = 0;
//   virtual bool addState(const BallState& pose) = 0;

  /// Get state at specific time.
  virtual bool getAt(const Scalar t, BallState* const state) = 0;

  /// Check if estimator is healthy.
  virtual bool healthy() const = 0;

  /// Get state at time state.t.
  bool getState(BallState* const state);

  /// Get latest state.
  bool getRecent(BallState* const state);
  BallState getRecent();
};

}  // namespace agi