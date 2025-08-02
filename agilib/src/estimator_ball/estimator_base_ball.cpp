#include "agilib/estimator_ball/estimator_base_ball.hpp"

namespace agi {

EstimatorBaseBall::EstimatorBaseBall(const std::string& name) : Module(name) {}

EstimatorBaseBall::~EstimatorBaseBall() {}

bool EstimatorBaseBall::getState(BallState* const state) {
  if (state == nullptr) return false;
  return getAt(state->t, state);
}

bool EstimatorBaseBall::getRecent(BallState* const state) {
  if (state == nullptr) return false;
  return getAt(-1.0, state);
}

BallState EstimatorBaseBall::getRecent() {
  BallState state;
  getAt(-1.0, &state);
  return state;
}

}  // namespace agi
