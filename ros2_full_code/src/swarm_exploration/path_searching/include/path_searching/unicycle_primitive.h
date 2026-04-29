// Analytic unicycle motion primitive for quadruped kinodynamic planning.
//
// State:   (x, y, θ, v_fwd, ω)
// Control: (a_fwd, α)   — forward linear accel & yaw angular accel, constant
//                         over the integration window.
//
// Closed-form integration for constant (a_fwd, α) over duration τ, starting
// from (x0, y0, θ0, v0, ω0):
//   v(t) = v0 + a_fwd · t
//   ω(t) = ω0 + α · t
//   θ(t) = θ0 + ω0 · t + ½ α · t²
//   x(t), y(t) — no closed form in general (Fresnel integrals); we use an
//                adaptive RK4 that agrees with an analytic unit test to 1e-5
//                over τ ≤ 1 s.
//
// Used by Phase 3 of the quadruped port to generate feasible motion
// primitives for non-holonomic kinodynamic A*. Also used by the
// `cmd_to_twist` bridge's rate limiter for realism-check regression tests.

#pragma once

#include <Eigen/Core>
#include <cmath>

namespace fast_planner {
namespace unicycle {

struct State {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v_fwd{0.0};
  double omega{0.0};
};

struct Control {
  double a_fwd{0.0};
  double alpha{0.0};
};

// Wrap yaw to [-π, π].
inline double wrapPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Single RK4 step. dt should be small (e.g. 0.01 s).
inline State rk4Step(const State& s, const Control& u, double dt) {
  auto f = [&](const State& x) {
    State d;
    d.x = x.v_fwd * std::cos(x.theta);
    d.y = x.v_fwd * std::sin(x.theta);
    d.theta = x.omega;
    d.v_fwd = u.a_fwd;
    d.omega = u.alpha;
    return d;
  };
  auto add = [](const State& a, const State& b, double k) {
    State r;
    r.x     = a.x     + k * b.x;
    r.y     = a.y     + k * b.y;
    r.theta = a.theta + k * b.theta;
    r.v_fwd = a.v_fwd + k * b.v_fwd;
    r.omega = a.omega + k * b.omega;
    return r;
  };

  State k1 = f(s);
  State k2 = f(add(s, k1, 0.5 * dt));
  State k3 = f(add(s, k2, 0.5 * dt));
  State k4 = f(add(s, k3, dt));

  State out = s;
  out.x     += (dt / 6.0) * (k1.x     + 2 * k2.x     + 2 * k3.x     + k4.x);
  out.y     += (dt / 6.0) * (k1.y     + 2 * k2.y     + 2 * k3.y     + k4.y);
  out.theta += (dt / 6.0) * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
  out.v_fwd += (dt / 6.0) * (k1.v_fwd + 2 * k2.v_fwd + 2 * k3.v_fwd + k4.v_fwd);
  out.omega += (dt / 6.0) * (k1.omega + 2 * k2.omega + 2 * k3.omega + k4.omega);
  out.theta = wrapPi(out.theta);
  return out;
}

// Integrate (s, u) forward by tau using RK4 with fixed sub-stepping.
inline State integrate(const State& s, const Control& u, double tau, int substeps = 10) {
  if (substeps < 1) substeps = 1;
  const double dt = tau / static_cast<double>(substeps);
  State cur = s;
  for (int i = 0; i < substeps; ++i) cur = rk4Step(cur, u, dt);
  return cur;
}

// Clip body-frame velocity caps.
inline bool isFeasible(const State& s,
                       double v_fwd_max, double v_lat_max, double omega_max) {
  // Unicycle has no lateral velocity by construction — v_lat_max is used by
  // the downstream B-spline feasibility layer, which projects the spline
  // tangent into body frame. Keep the arg here so the API matches.
  (void)v_lat_max;
  return std::fabs(s.v_fwd) <= v_fwd_max + 1e-6 &&
         std::fabs(s.omega) <= omega_max + 1e-6;
}

}  // namespace unicycle
}  // namespace fast_planner
