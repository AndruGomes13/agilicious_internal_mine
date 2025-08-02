#include <iostream>
#include "agilib/estimator_ball/estimator_base_ball.hpp"
#include "agilib/math/types.hpp"
#include "agilib/types/ball.hpp"
#include "agilib/types/ball_state.hpp"
#include "agilib/estimator_ball/ekf_ball/ekf_ball.hpp"


namespace agi {

EkfBall::EkfBall(const Ball& ball, const std::shared_ptr<EkfParametersBall>& params)
  : EstimatorBaseBall("EKF_Ball"),
    ball_(ball),
    integrator_(ball_.getDynamicsFunction()) {
  if (!updateParameters(params)) logger_.error("Could not load parameters!");
}

EkfBall::~EkfBall() { printTimings(); }

bool EkfBall::getAt(const Scalar t, BallState* const state) {
  if (state == nullptr) return false;
  if (std::isnan(t)) return false;
  if (!posterior_.valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (params_->update_on_get) process();

  // Catch trivial cases...
  if (t <= prior_.t) {  // ...request at prior
    *state = prior_;
    return true;
    // return false;  // Not allowed to get prior state. This is because the prior is not updated with empty frames and thus can be outdated.
  }

  const bool ret = propagatePrior(t);
  *state = prior_;
  state->seen_last_frame = last_processed_frame_had_point;
  state->last_seen_t = t_last_processed_frame_with_point;

  return ret;
}

bool EkfBall::initialize(const BallState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  return init(state);
}

bool EkfBall::init(const BallState& state) {
  if (!state.valid()) return false;

  if (!frames_.empty()) {
    const std::deque<Frame>::iterator first_frame = std::lower_bound(
      frames_.begin(), frames_.end(), state.t,
      [](const Frame& frame, const Scalar t) { return  frame.t < t; });

    frames_.erase(frames_.begin(), first_frame);
  }

  prior_ = state;
  posterior_ = state;
  P_ = Q_init_;

  return true;
}


bool EkfBall::addFrame(const Frame& frame) {
  if (!frame.point.has_value()){
    if (!posterior_.valid()) {
      // If the filter is not initialized, no need to add a frame without a point.
      return true;
    }
    frames_.push_back(frame);
  }else{
    Point point = frame.point.value();
    if (!point.valid()) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    if (!posterior_.valid()) {
      logger_.info("Posterior is not valid, initializing with point.");
      posterior_.setZero();
      posterior_.t = point.t;
      posterior_.seen_last_frame = true;
      posterior_.last_seen_t = point.t;
      posterior_.p = point.position;
      t_last_processed_frame_with_point = point.t;
      return init(posterior_);
    }

    if (point.t <= posterior_.t) return false;

    frames_.push_back(frame);
    t_last_point = point.t;
  }

  while ((int)frames_.size() > MAX_QUEUE_SIZE) frames_.pop_front();

  process();
  return true;

}

bool EkfBall::addPointCloud(const PointCloud& point_cloud) {
    int num_points = point_cloud.size();

    if (num_points == 0) {
        logger_.warn("Received empty point cloud, skipping.");
        addFrame(Frame{
            .t = point_cloud.t,
            .point = std::nullopt,  // No point update
        });
    } else if (num_points == 1) {
        // If we have only one point, we can use it as a point update.
        Point point = point_cloud.points[0];
        point.t = point_cloud.t;
        addFrame(Frame{
            .t = point.t,
            .point = point,
        });
    } else {
        // If we have multiple points, we can use the one closest to the prior.
        Point closest_point = point_cloud.points[0];
        Scalar min_distance = (closest_point.position - prior_.p).norm();
        for (const auto& p : point_cloud.points) {
            Scalar distance = (p.position - prior_.p).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = p;
            }
        }
        addFrame(Frame{
            .t = closest_point.t,
            .point = closest_point,
        });
    }
    return true;
}

bool EkfBall::healthy() const { return posterior_.valid() && prior_.valid(); }


void EkfBall::logTiming() const {
  logger_.addPublishingVariable(
    "timer_process_ms",
    1000.0 * Vector<3>(timer_process_.mean(), timer_process_.min(),
                       timer_process_.max()));
}

void EkfBall::printTimings(const bool all) const {
  logger_ << timer_process_;
  if (all) {
    logger_ << timer_update_point_;
    logger_ << timer_compute_gain_;
    logger_ << timer_propagation_;
    logger_ << timer_jacobian_;
  }
}

bool EkfBall::process() {
  if (params_->enable_timing) {timer_process_.tic();}
  std::deque<Frame>::iterator frame = frames_.begin();

  while (frame < frames_.end()) {  // Iterator over all frames in queue

    if (!frame->point.has_value()){
        // If it's an empty frame.
        last_processed_frame_had_point = false;
        
      } else {
      Point point = frame->point.value();
        if (!updatePoint(point)){
            logger_.error("Could not perform point update at %1.3f",
                        point.t);
                        }
        last_processed_frame_had_point = true;
      }
      // Pedantic: exit if queues have been emptied in possible filter re-init.
      if (frames_.empty()) {return true;}
      frame++;
  }

  // Remove what was processed, pedantic checking for end-iterator change.
  if (frame > frames_.end()) {frame = frames_.end();}
  frames_.erase(frames_.begin(), frame);

  if (params_->enable_timing) {timer_process_.toc();}
  return true;
}

bool EkfBall::updatePoint(const Point& point) {
  if (params_->enable_timing) timer_update_point_.tic();
  if (!propagatePriorAndJacobian(point.t)) {
    logger_.error("Could not propagate to %1.3f", point.t);
    return false;
  }

  if (params_->jump_pos_threshold > 0.0) {
    const Vector<3> d_pos = point.position - prior_.p;
    const bool pos_jump = d_pos.norm() > params_->jump_pos_threshold;

    if (pos_jump) {
      logger_.warn(
        "Detected jump in point measurement!\n"
        "Time: %1.3f\n"
        "Position:  %1.1f m\n"
        "Angle:     %1.1f rad\n",
        point.t, d_pos.norm());
      if (pos_jump) {
        logger_ << "Prior Pos: " << prior_.p.transpose() << std::endl;
        logger_ << "Meas Pos:  " << point.position.transpose() << std::endl;
      }

      posterior_.setZero();
      posterior_.t = point.t;
      posterior_.p = point.position;
      posterior_.seen_last_frame = true;
      posterior_.last_seen_t = point.t;
      t_last_processed_frame_with_point = point.t;
      return init(posterior_);
    }
  }

  if (point.t - t_last_processed_frame_with_point > params_->max_unseen_wait_time) {
    // The last seen point was too long ago, reset the filter.
    logger_.warn("Point update at %1.3f is too old, resetting filter.", point.t);
    posterior_.setZero();
    posterior_.t = point.t;
    posterior_.p = point.position;
    posterior_.seen_last_frame = true;
    posterior_.last_seen_t = point.t;
    t_last_processed_frame_with_point = point.t;
    return init(posterior_);
  }

  // Point update residual and jacobian.
  const Vector<SRPOS> y = prior_.p - point.position;   // done

  Matrix<SRPOS, BallState::SIZE> H = Matrix<SRPOS, BallState::SIZE>::Zero();
  H.block<3, 3>(0, BallState::IDX::POS) = Matrix<3, 3>::Identity();

  const bool check = update(y, H, R_pos_);
  posterior_.last_seen_t = point.t;
  posterior_.seen_last_frame = true;
  t_last_processed_frame_with_point = point.t;
  if (params_->enable_timing) timer_update_point_.toc();
  return check;
}

bool EkfBall::propagatePrior(const Scalar t) {
  if (std::isnan(t)) return false;

  if (params_->enable_timing) timer_propagation_.tic();

  if (t < prior_.t) {
    if (t < posterior_.t) return false;
    prior_ = posterior_;
  }

  const BallState initial = prior_;
  prior_.t = t;
  logger_.info("Integrating dt: %1.3f", prior_.t - initial.t);
  logger_.info("Prior t: %1.3f", prior_.t);
  logger_.info("Initial t: %1.3f", initial.t);

  integrator_.integrate(initial, &prior_);
  sanitize(prior_);

  if (params_->enable_timing) timer_propagation_.toc();
  return true;
}

bool EkfBall::propagatePriorAndJacobian(const Scalar t) {
  if (std::isnan(t) || t < posterior_.t) return false;

  if (params_->enable_timing) timer_jacobian_.tic();

  prior_ = posterior_;

  sanitize(prior_);

  const Scalar dt_max = integrator_.dtMax();
  Scalar dt_remaining = t - prior_.t;
  Vector<BallState::SIZE> propagated_state;
  SparseMatrix F(BallState::SIZE, BallState::SIZE);

  while (dt_remaining > 0.0) {
    const Scalar dt = std::min(dt_remaining, dt_max);

    ball_.jacobian(prior_.x, F);
    const Matrix<BallState::SIZE, BallState::SIZE> P = P_;
    F *= dt;
    P_ += F * P;
    P_ += P * F.transpose();
    P_ += dt * Q_;

    integrator_.step(prior_.x, dt, propagated_state);

    prior_.x = propagated_state;
    prior_.t += dt;
    sanitize(prior_);
    dt_remaining -= dt;
  }

  prior_.t = t;
  sanitize(prior_);
  ;

  if (params_->enable_timing) timer_jacobian_.toc();
  return true;
}

void EkfBall::sanitize(BallState& state) {
    // Since the state does not have a quaternion, nothing to do here.
}

bool EkfBall::updateParameters(const std::shared_ptr<EkfParametersBall>& params) {
  if (!params || !params->valid()) return false;

  std::lock_guard<std::mutex> lock(mutex_);

  params_ = params;

  Q_.setZero();
  Q_init_.setZero();
  R_pos_.setZero();
 
  // Process noise
  const Vector<> q = (Vector<BallState::SIZE>() << params_->Q_pos, params_->Q_vel).finished();

  Matrix<BallState::SIZE, BallState::SIZE> Q = Matrix<BallState::SIZE, BallState::SIZE>::Zero();
  Q.topLeftCorner(q.rows(), q.rows()) = q.asDiagonal();
  Q_ = Q.sparseView();

  const Vector<> q_init =
    (Vector<BallState::SIZE>() << params_->Q_init_pos, params_->Q_init_vel).finished();

  Matrix<BallState::SIZE, BallState::SIZE> Q_init = Matrix<BallState::SIZE, BallState::SIZE>::Zero();
  Q_init.topLeftCorner(q_init.rows(), q_init.rows()) = q_init.asDiagonal();
  Q_init_ = Q_init.sparseView();

  R_pos_ = (Vector<SRPOS>() << params_->R_pos)
              .finished()
              .asDiagonal();

  return true;
}

}