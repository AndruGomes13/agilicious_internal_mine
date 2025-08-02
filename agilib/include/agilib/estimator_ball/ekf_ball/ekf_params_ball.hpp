
#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/math/types.hpp"
#include "agilib/utils/logger.hpp"

namespace agi {

struct EkfParametersBall : public ParameterBase {
  EkfParametersBall();

  using ParameterBase::load;
  bool load(const Yaml& node) override;

  bool valid() const override;

  bool enable_timing;

  bool update_on_get;
  Scalar jump_pos_threshold;
  Scalar max_unseen_wait_time;

  // Measurement noise
  Vector<3> R_pos;

  // Process noise
  Vector<3> Q_pos;
  Vector<3> Q_vel;

  Vector<3> Q_init_pos;
  Vector<3> Q_init_vel;
};

}  // namespace agi
