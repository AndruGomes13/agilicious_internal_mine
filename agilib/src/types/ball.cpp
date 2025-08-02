#include "agilib/types/ball.hpp"
#include "agilib/math/gravity.hpp"
#include "agilib/types/ball_state.hpp"

namespace agi {

Ball::Ball(const Scalar mass,
           const Scalar drag_coeff,
           const Scalar restitution,
           const Scalar ground_z)
    : m_(mass),
      c_(drag_coeff),
      e_(restitution),
      z_ground_(ground_z) {}

/* ---------- f(x) -------------------------------------------------------- */
bool Ball::dynamics(const BallState& state,
                    BallState* const derivative) const {
  if (derivative == nullptr) return false;
  return dynamics(state.x, derivative->x);
}

bool Ball::dynamics(const Ref<const Vector<BallState::SIZE>> state,
                    Ref<Vector<BallState::SIZE>> deriv) const {
  if (!state.segment<BallState::DYN>(0).allFinite()) return false;

  deriv.setZero();

  /* ṗ = v */
  deriv.segment<BallState::NPOS>(BallState::POS) =
      state.segment<BallState::NVEL>(BallState::VEL);

  /* v̇ = g + drag */
  const Vector<3> v = state.segment<BallState::NVEL>(BallState::VEL);
  const Scalar    vnorm = v.norm();
  Vector<3> a = GVEC      ;          // gravity
  if (vnorm > 1e-6)                                      // quadratic drag
    a += -(c_ / m_) * vnorm * v;
  deriv.template segment<3>(BallState::VELX) = a;

  return true;
}

/* ---------- ∂f/∂x ------------------------------------------------------- */
bool Ball::jacobian(const Ref<const Vector<BallState::SIZE>> state,
                    SparseMatrix& jac) const {
  Matrix<BallState::SIZE, BallState::SIZE> jac_dense;
  const bool ret = jacobian(state, jac_dense);
  jac = jac_dense.sparseView();
  return ret;
}

bool Ball::jacobian(const Ref<const Vector<BallState::SIZE>> state,
                    Ref<      Matrix<BallState::SIZE, BallState::SIZE>> jac) const {
  if (!state.segment<BallState::DYN>(0).allFinite()) return false;

  jac.setZero();

  /* ∂ṗ/∂v = I */
  jac.block<BallState::NPOS,BallState::NVEL>(BallState::POS, BallState::VEL) =
      Matrix<3,3>::Identity();

  /* ∂v̇/∂v = −(c/m)(‖v‖I + vvᵀ / ‖v‖) */ //TODO: Check if this is correct
  const Vector<3> v = state.segment<BallState::NVEL>(BallState::VEL);
  const Scalar    vnorm = v.norm();
  if (vnorm > 1e-6) {
    jac.block<BallState::NVEL,BallState::NVEL>(BallState::VEL, BallState::VEL) =
        -(c_ / m_) *
        ((vnorm * Matrix<3,3>::Identity()) +
         (v * v.transpose()) / vnorm);
  }

  return true;
}

/* ---------- helper ------------------------------------------------------ */
DynamicsFunction Ball::getDynamicsFunction() const {
  using std::placeholders::_1;
  using std::placeholders::_2;
  return std::bind(static_cast<bool (Ball::*)(
                     const Ref<const Vector<BallState::SIZE>>,
                     Ref<Vector<BallState::SIZE>>) const>(&Ball::dynamics),
                   this, _1, _2);
}

}  // namespace agi