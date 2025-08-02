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
  logger_.info("Processing.");
  if (params_->update_on_get) process();

  // Catch trivial cases...
  if (t <= prior_.t) {  // ...request at prior
    *state = prior_;
    return true;
    // return false;  // Not allowed to get prior state. This is because the prior is not updated with empty frames and thus can be outdated.
  }

  logger_.info("Propagating prior.");
  const bool ret = propagatePrior(t);
  *state = prior_;
  state->seen_last_frame = last_processed_frame_had_pose;
  state->last_seen_t = t_last_processed_frame_with_pose;

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
  if (!frame.pose.has_value()){
    if (!posterior_.valid()) {
      // If the filter is not initialized, no need to add a frame without a pose.
      return true;
    }
    logger_.info("Adding empty frame at time %1.3f", frame.t);
    frames_.push_back(frame);
  }else{
    Pose pose = frame.pose.value();
    if (!pose.valid()) return false;

    std::lock_guard<std::mutex> lock(mutex_);

    if (!posterior_.valid()) {
      posterior_.setZero();
      posterior_.t = pose.t;
      posterior_.p = pose.position;
      return init(posterior_);
    }

    if (pose.t <= posterior_.t) return false;

    frames_.push_back(frame);
    t_last_pose_ = pose.t;
  }

  while ((int)frames_.size() > MAX_QUEUE_SIZE) frames_.pop_front();

  process();
  return true;

}

bool EkfBall::addPointCloud(const PointCloud& point_cloud) {
    int num_points = point_cloud.size();
    logger_.info("Received PointCloud with %d points at time %1.3f", num_points, point_cloud.t);

    if (num_points == 0) {
        logger_.warn("Received empty point cloud, skipping.");
        addFrame(Frame{
            .t = point_cloud.t,
            .pose = std::nullopt,  // No pose update
        });
    } else if (num_points == 1) {
        // If we have only one point, we can use it as a pose update.
        Pose pose = point_cloud.poses[0];
        pose.t = point_cloud.t;
        addFrame(Frame{
            .t = pose.t,
            .pose = pose,
        });
    } else {
        // If we have multiple points, we can use the one closest to the prior.
        Pose closest_pose = point_cloud.poses[0];
        Scalar min_distance = (closest_pose.position - prior_.p).norm();
        for (const auto& p : point_cloud.poses) {
            Scalar distance = (p.position - prior_.p).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_pose = p;
            }
        }
        addFrame(Frame{
            .t = closest_pose.t,
            .pose = closest_pose,
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
    logger_ << timer_update_pose_;
    logger_ << timer_compute_gain_;
    logger_ << timer_propagation_;
    logger_ << timer_jacobian_;
  }
}

bool EkfBall::process() {
  if (params_->enable_timing) {timer_process_.tic();}
  std::deque<Frame>::iterator frame = frames_.begin();

  while (frame < frames_.end()) {  // Iterator over all frames in queue

    if (!frame->pose.has_value()){
        // If it's an empty frame.
        last_processed_frame_had_pose = false;
        
      } else {
      Pose pose = frame->pose.value();
        if (!updatePose(pose)){
            logger_.error("Could not perform pose update at %1.3f",
                        pose.t);
                        }
        last_processed_frame_had_pose = true;
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

bool EkfBall::updatePose(const Pose& pose) {
  if (params_->enable_timing) timer_update_pose_.tic();
  if (!propagatePriorAndJacobian(pose.t)) {
    logger_.error("Could not propagate to %1.3f", pose.t);
    return false;
  }

  if (params_->jump_pos_threshold > 0.0) {
    const Vector<3> d_pos = pose.position - prior_.p;
    const bool pos_jump = params_->jump_pos_threshold > 0.0 &&
                          d_pos.norm() > params_->jump_pos_threshold;

    if (pos_jump) {
      logger_.warn(
        "Detected jump in pose measurement!\n"
        "Time: %1.3f\n"
        "Position:  %1.1f m\n"
        "Angle:     %1.1f rad\n",
        pose.t, d_pos.norm());
      if (pos_jump) {
        logger_ << "Prior Pos: " << prior_.p.transpose() << std::endl;
        logger_ << "Meas Pos:  " << pose.position.transpose() << std::endl;
      }

      posterior_.setZero();
      posterior_.t = pose.t;
      posterior_.p = pose.position;
      return init(posterior_);
    }
  }

  if (pose.t - t_last_processed_frame_with_pose > params_->max_unseen_wait_time) {
    // The last seen pose was too long ago, reset the filter.
    logger_.warn("Pose update at %1.3f is too old, resetting filter.", pose.t);
    posterior_.setZero();
    posterior_.t = pose.t;
    posterior_.p = pose.position;
    return init(posterior_);
  }

  // Pose update residual and jacobian.
  const Vector<SRPOS> y = prior_.p - pose.position;   // done

  Matrix<SRPOS, BallState::SIZE> H = Matrix<SRPOS, BallState::SIZE>::Zero();
  H.block<3, 3>(0, BallState::IDX::POS) = Matrix<3, 3>::Identity();

  const bool check = update(y, H, R_pose_);
  posterior_.last_seen_t = pose.t;
  posterior_.seen_last_frame = true;
  t_last_processed_frame_with_pose = pose.t;
  if (params_->enable_timing) timer_update_pose_.toc();
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
  R_pose_.setZero();
 
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

  R_pose_ = (Vector<SRPOS>() << params_->R_pos)
              .finished()
              .asDiagonal();

  return true;
}

}