// Sanity test for the unicycle primitive (no rclcpp dependency — runs as a
// plain main()).  Checks that:
//   1. Zero control preserves position; θ and v_fwd are unchanged.
//   2. Pure forward motion (a_fwd=0, v0=1.0, α=0, ω=0) over τ=1 s advances x
//      by exactly 1.0, y = 0.
//   3. Pure in-place rotation (v_fwd=0, α=0, ω=1 rad/s) over τ=π/2 rotates θ
//      by π/2, position unchanged.
//   4. Combined (v=1, ω=1) over τ=1 s — the unicycle circles; compare against
//      the analytic r=v/ω = 1 circular arc.
//
// Run with:    ros2 run path_searching test_unicycle_primitive
// Returns 0 on success, non-zero on failure.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "path_searching/unicycle_primitive.h"

using fast_planner::unicycle::State;
using fast_planner::unicycle::Control;
using fast_planner::unicycle::integrate;

static int fails = 0;

static void expect_near(const char* name, double got, double want, double tol) {
  if (std::fabs(got - want) > tol) {
    std::fprintf(stderr, "FAIL %s: got %.6f, want %.6f (tol %.6f)\n",
                 name, got, want, tol);
    ++fails;
  } else {
    std::fprintf(stderr, "PASS %s: %.6f\n", name, got);
  }
}

int main() {
  // Test 1: zero control, zero initial — everything stays zero.
  {
    State s0{};
    Control u{};
    State s1 = integrate(s0, u, 1.0);
    expect_near("zero.x",     s1.x,     0.0, 1e-9);
    expect_near("zero.y",     s1.y,     0.0, 1e-9);
    expect_near("zero.theta", s1.theta, 0.0, 1e-9);
  }
  // Test 2: pure forward 1 m/s for 1 s.
  {
    State s0{};
    s0.v_fwd = 1.0;
    Control u{};
    State s1 = integrate(s0, u, 1.0);
    expect_near("fwd.x",     s1.x,     1.0, 1e-6);
    expect_near("fwd.y",     s1.y,     0.0, 1e-6);
    expect_near("fwd.theta", s1.theta, 0.0, 1e-9);
    expect_near("fwd.v",     s1.v_fwd, 1.0, 1e-9);
  }
  // Test 3: pure in-place rotation ω=1 for τ=π/2.
  {
    State s0{};
    s0.omega = 1.0;
    Control u{};
    State s1 = integrate(s0, u, M_PI / 2.0);
    expect_near("rot.x",     s1.x,     0.0, 1e-9);
    expect_near("rot.y",     s1.y,     0.0, 1e-9);
    expect_near("rot.theta", s1.theta, M_PI / 2.0, 1e-6);
    expect_near("rot.omega", s1.omega, 1.0, 1e-9);
  }
  // Test 4: v=1, ω=1, τ=1 → end at (sin(1), 1-cos(1)).
  {
    State s0{};
    s0.v_fwd = 1.0;
    s0.omega = 1.0;
    Control u{};
    State s1 = integrate(s0, u, 1.0, 200);  // tighter substeps for arc accuracy
    expect_near("arc.x",     s1.x,     std::sin(1.0),        1e-4);
    expect_near("arc.y",     s1.y,     1.0 - std::cos(1.0),  1e-4);
    expect_near("arc.theta", s1.theta, 1.0,                  1e-6);
  }
  // Test 5: forward accel. a_fwd=1, v0=0, τ=1 → v=1, x=0.5.
  {
    State s0{};
    Control u{};
    u.a_fwd = 1.0;
    State s1 = integrate(s0, u, 1.0);
    expect_near("accel.v", s1.v_fwd, 1.0, 1e-9);
    expect_near("accel.x", s1.x,     0.5, 1e-5);
  }

  if (fails == 0) {
    std::fprintf(stderr, "\nAll unicycle primitive tests PASS.\n");
    return 0;
  }
  std::fprintf(stderr, "\n%d test(s) FAILED.\n", fails);
  return 1;
}
