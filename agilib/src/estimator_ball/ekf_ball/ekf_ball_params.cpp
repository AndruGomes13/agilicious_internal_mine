#include "agilib/estimator_ball/ekf_ball/ekf_params_ball.hpp"

namespace agi {

EkfParametersBall::EkfParametersBall()
  : enable_timing(true),
   update_on_get(false),
   jump_pos_threshold(0.0),
   max_unseen_wait_time(0.5),
    R_pos(Vector<3>::Constant(1e-6)),
    Q_pos(Vector<3>::Constant(1e-6)),
    Q_vel(Vector<3>::Constant(1e-3)),
    Q_init_pos(Vector<3>::Constant(1e-6)),
    Q_init_vel(Vector<3>::Constant(1e-3)){}


bool EkfParametersBall::load(const Yaml& node) {
  if (node.isNull()) return false;

  enable_timing = node["enable_timing"].as<bool>();
  max_unseen_wait_time = node["max_unseen_wait_time"].as<Scalar>();
  update_on_get = node["update_on_get"].as<bool>();
  jump_pos_threshold = node["jump_pos_threshold"].as<Scalar>();
  node["R_pos"] >> R_pos;
  node["Q_pos"] >> Q_pos;
  node["Q_vel"] >> Q_vel;
  node["Q_init_pos"] >> Q_init_pos;
  node["Q_init_vel"] >> Q_init_vel;

  return valid();
}

bool EkfParametersBall::valid() const {
  bool check = true;

  check &= std::isfinite(max_unseen_wait_time);

  check &= R_pos.allFinite();

  check &= Q_pos.allFinite();
  check &= Q_vel.allFinite();

  check &= Q_init_pos.allFinite();
  check &= Q_init_vel.allFinite();

  return check;
}

}  // namespace agi
