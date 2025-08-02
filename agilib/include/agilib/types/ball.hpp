#pragma once
#include "agilib/types/ball_state.hpp"

namespace agi {

/**
 * Simple ballistic (free-flight) rigid body with quadratic drag.
 * State:  p, v   (6×1)
 *
 *   ṗ = v
 *   v̇ = g·e_z  –  (c/m)‖v‖ v
 *
 * A restitution coefficient e_ is exposed so you can implement a
 * discrete “bounce” correction in your EKF predict-or-reset logic.
 */
class Ball {
 public:
  explicit Ball(Scalar mass           = 0.055,   // [kg] -- tennis-ball ish
                Scalar drag_coeff     = 0.45,    // [kg/m]  (ρA C_d /2 lumped)
                Scalar restitution    = 0.75,    // unitless
                Scalar ground_z       = 0.0);    // world frame ground plane

  /* continuous dynamics --------------------------------------------------- */
  bool dynamics(const BallState& state,
                BallState* const derivative) const;

  bool dynamics(const Ref<const Vector<BallState::SIZE>> state,
                Ref<      Vector<BallState::SIZE>> deriv) const;

  bool jacobian(const Ref<const Vector<BallState::SIZE>> state,
                         SparseMatrix& jac) const;

  bool jacobian(const Ref<const Vector<BallState::SIZE>> state,
                Ref<      Matrix<BallState::SIZE, BallState::SIZE>> jac) const;


  /* plug-in for integrator ------------------------------------------------- */
  DynamicsFunction getDynamicsFunction() const;

  /* misc setters ----------------------------------------------------------- */
  void setDragCoeff(Scalar c)      { c_ = c; }
  void setRestitution(Scalar e)    { e_ = e; }
  void setGroundPlane(Scalar z_g)  { z_ground_ = z_g; }

  Scalar restitution()   const { return e_; }
  Scalar groundPlane()   const { return z_ground_; }

 private:
  Scalar m_;          // mass [kg]
  Scalar c_;          // lumped ball-drag coeff  (ρA C_d /2) [kg/m]
  Scalar e_;          // bounce restitution
  Scalar z_ground_;   // ground-plane height [m]
};

}  // namespace agi