#pragma once

#include <deque>
#include <mutex>

#include "agilib/estimator_ball/ekf_ball/ekf_params_ball.hpp"
#include "agilib/estimator_ball/estimator_base_ball.hpp"
#include "agilib/math/integrator_rk4.hpp"
#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/types/point.hpp"
#include "agilib/types/ball_state.hpp"
#include "agilib/types/ball.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {
  

/// EKF filter for point measurements.
class EkfBall : public EstimatorBaseBall {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EkfBall(const Ball& ball, const std::shared_ptr<EkfParametersBall>& params =
                               std::shared_ptr<EkfParametersBall>());
  ~EkfBall();

  bool getAt(const Scalar t, BallState* const state) override;

  bool initialize(const BallState& state) override;

  bool addFrame(const Frame& frame);
  bool addPointCloud(const PointCloud& point_cloud) override;

  bool updateParameters(const std::shared_ptr<EkfParametersBall>& params);

  bool healthy() const override;

  inline void printTiming() { printTimings(false); }
  void printTimings(const bool all = true) const;
  void logTiming() const override;

 private:
  bool init(const BallState& state);
  bool process();
  template<int N>
  bool update(const Vector<N>& y, const Matrix<N, BallState::SIZE>& H,
              const Matrix<N, N>& R);
  bool updatePoint(const Point& point);
  bool propagatePrior(const Scalar time);
  bool propagatePriorAndJacobian(const Scalar time);
  void sanitize(BallState& state);

  static constexpr int MAX_QUEUE_SIZE = 256;
  static constexpr int SRPOS = 3;  // Size of position measurement.

  // Parameters
  std::shared_ptr<EkfParametersBall> params_;
  SparseMatrix Q_{BallState::SIZE, BallState::SIZE};
  SparseMatrix Q_init_{BallState::SIZE, BallState::SIZE};
  Matrix<SRPOS, SRPOS> R_pos_;
  Ball ball_;
  IntegratorRK4 integrator_;

  // Working variables
  BallState posterior_;
  BallState prior_;
  Matrix<BallState::SIZE, BallState::SIZE> P_;

  // Thread Safety
  std::mutex mutex_;

  std::deque<Frame> frames_;
  Scalar t_last_point{NAN};
  bool last_processed_frame_had_point{false};
  Scalar t_last_processed_frame_with_point{NAN};


  // Logger and timers
  Timer timer_process_{"Process"};
  Timer timer_update_point_{"Update point"};
  Timer timer_compute_gain_{"Kalman gain computation"};
  Timer timer_propagation_{"Propagation Prior"};
  Timer timer_jacobian_{"Propagation Jacobian"};
};


template<int N>
bool EkfBall::update(const Vector<N>& y, const Matrix<N, BallState::SIZE>& H,
                 const Matrix<N, N>& R) {
  if (params_->enable_timing) timer_compute_gain_.tic();

  const Matrix<BallState::SIZE, BallState::SIZE> P = 0.5 * (P_ + P_.transpose());
  const Matrix<N, BallState::SIZE> HP = H * P;
  const Matrix<N, N> S = HP * H.transpose() + R;
  const Matrix<N, N> S_inv = S.inverse();

  if (!S_inv.allFinite()) return false;

  BallState new_posterior = prior_;

  const Matrix<BallState::SIZE, N> K = P * H.transpose() * S_inv;
  new_posterior.x -= K * y;

  P_ = P - K * HP;

  if (!new_posterior.valid()) return false;

  posterior_ = new_posterior;
  prior_ = new_posterior;

  if (params_->enable_timing) timer_compute_gain_.toc();
  return true;
}

}  // namespace agi
