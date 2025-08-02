#pragma once
#include <ostream>
#include "agilib/math/math.hpp"   // gives Scalar, Vector<>, etc.
#include "agilib/math/types.hpp"
#include <memory>

namespace agi {

/**
 *  x = [ p_x p_y p_z v_x v_y v_z ]ᵀ
 */
struct BallState {
  enum IDX : int{ 
    POS = 0,
    POSX = 0, 
    POSY = 1, 
    POSZ = 2, 
    NPOS = 3, 
    VEL = 4, 
    VELX = 4, 
    VELY = 5, 
    VELZ = 6, 
    NVEL = 3, 
    DYN = 6,
    SIZE = 6
  };

  Scalar t{NAN};
  bool seen_last_frame {false};
  Scalar last_seen_t {NAN};
  Vector<IDX::SIZE> x{Vector<IDX::SIZE>::Constant(NAN)};

  BallState() = default;
  BallState(const Scalar t, const Vector<IDX::SIZE>& x) : t(t), x(x) {}
  BallState(Scalar t, const Vector<3>& pos, const Vector<3>& vel ) : t(t) {
    p = pos;
    v = vel;
  }

  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};

  /* utils ------------------------------------------------------------------ */
  void setZero(bool reset_time = true) {
    if (reset_time) t = 0.0;
    x.setZero();
  }

  // inline bool valid() const { return x.allFinite() && std::isfinite(t) && t - last_seen_t < 1; } // TODO: make this a parameter
  inline bool valid() const { return x.allFinite() && std::isfinite(t); } // TODO: make this a parameter

  friend inline std::ostream& operator<<(std::ostream& os, const BallState& s) {
    os.precision(6);
    os << std::scientific;
    os << "Ball at " << s.t << " s:\n;";
    os.precision(3);
    os << "p= [" << s.p.transpose() << "]\n"
       << "v= [" << s.v.transpose() << "]" << std::endl;
    os.precision();
    return os;
  }

};


}  // namespace agi