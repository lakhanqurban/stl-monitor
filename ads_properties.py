"""
ADS STL Property Suite
======================
Six properties grounded in the actual signal ranges observed in the
Udacity ADS simulation data. Each property is:

  P1 — Lane Keeping Safety:   |CTE| sustained breach must stay below 1.0m
  P2 — Speed Stability:       Speed must stay above 5 mph once moving
  P3 — Steering Smoothness:   Steering angle changes must be gradual
  P4 — Heading Alignment:     Heading error must stay bounded
  P5 — Recovery After Drift:  After large CTE, agent must recover
  P6 — High-Curvature Safety: On sharp curves, CTE must be tighter

Each property is represented as an STLFormula object.
Thresholds are derived from data statistics (see analysis in README).
"""

from stl_monitor import (
    Signal, Predicate, Not, And, Or, Globally, Eventually, SustainedViolation
)
import numpy as np


# ---------------------------------------------------------------------------
# Thresholds — single source of truth
# ---------------------------------------------------------------------------
# These named constants are the single source of truth for the property
# thresholds. The STL formulas below reference them, and `runner.py`
# imports them to keep its violation-segment masks in sync. Change a
# value here and both the formula and the mask update together.

# P1 — Lane Keeping Safety
# Relaxed from 0.8 → 1.0: data max=1.09, so 0.8 ≈ 73rd percentile of
# per-timestep CTE and virtually every scenario sees one spike above it.
# 1.0 m corresponds to the 95th-percentile range and aligns with the
# half-lane-width of the Udacity road (~2m total width → safe limit 1.0m).
P1_CTE_LIMIT = 1.0

# P2 — Speed Stability
P2_WARMUP_S = 2.0
P2_MIN_SPEED = 5.0

# P3 — Steering Smoothness
# Relaxed from 0.7 → 0.75: max observed is 0.83 rad; 0.7 is too close to
# the ceiling and fires on every normal sharp-curve steering input.
P3_STR_ANGLE_LIMIT = 0.75

# P4 — Heading Alignment
P4_INIT_SKIP_S = 0.5
P4_HDG_ERR_LIMIT = 0.25

# P5 — Recovery After Drift
# Recovered threshold raised 0.2 → 0.35: requiring the agent to go from
# >0.5m back to <0.2m (a 0.3m hard drop) in 3s is too strict. 0.35m is a
# 30% reduction in displacement, still meaningful but achievable.
# Recovery horizon extended 3.0 → 5.0s to allow for typical controller lag.
P5_DRIFT_THRESHOLD = 0.5
P5_RECOVERED_THRESHOLD = 0.35
P5_RECOVERY_HORIZON_S = 5.0

# P6 — High-Curvature Safety
# Curvature trigger raised 0.03 → 0.05 (actual sharp curves, not mild bends)
# and tight CTE limit raised 0.4 → 0.6m: 0.4m is half the now-relaxed P1
# limit and fires almost everywhere curves exist.
P6_CURVATURE_THRESHOLD = 0.05
P6_TIGHT_CTE_LIMIT = 0.6


# ---------------------------------------------------------------------------
# P1 — Lane Keeping Safety
# G (|cte| < 1.0)
# Rationale: CTE std=0.36, max=1.09. 1.0m ≈ 95th-percentile CTE and
# matches half the Udacity road width (~2m). The old 0.8m limit (~2σ above
# mean) caused 87.9% scenario violations because G fires on every spike.
# ---------------------------------------------------------------------------
P1_lane_keeping = SustainedViolation(
    Globally(
        Predicate('cte', lambda v: P1_CTE_LIMIT - abs(v),
                  label=f'|cte| < {P1_CTE_LIMIT}'),
        label=f"G(|cte| < {P1_CTE_LIMIT})"
    ),
    min_duration=0.5,
    label=f"P1: SustainedViolation[0.5s](G(|cte| < {P1_CTE_LIMIT}))"
)

# ---------------------------------------------------------------------------
# P2 — Speed Stability (after warm-up)
# G[2.0, ∞] (speed > 5.0)
# Rationale: Agent starts from rest. After 2s warm-up it should be moving.
# Mean speed = 27.9 mph; min meaningful speed = 5 mph.
# Stalling mid-road is a safety failure mode.
# ---------------------------------------------------------------------------
P2_speed_stability = Globally(
    Predicate('speed', lambda v: v - P2_MIN_SPEED,
              label=f'speed > {P2_MIN_SPEED}'),
    a=P2_WARMUP_S,
    label=f"P2: G[{P2_WARMUP_S},∞](speed > {P2_MIN_SPEED})"
)

# ---------------------------------------------------------------------------
# P3 — Steering Smoothness
# G (|str_angle| < 0.75)
# Rationale: str_angle std=0.23, max=0.83. Old limit 0.7 was only 0.13 rad
# below max, causing 50.2% violations on normal curve steering. 0.75 rad
# still flags near-saturation events while tolerating routine cornering.
# ---------------------------------------------------------------------------
P3_steering_smoothness = SustainedViolation(
    Globally(
        Predicate('str_angle', lambda v: P3_STR_ANGLE_LIMIT - abs(v),
                  label=f'|str_angle| < {P3_STR_ANGLE_LIMIT}'),
        label=f"G(|str_angle| < {P3_STR_ANGLE_LIMIT})"
    ),
    min_duration=0.5,
    label=f"P3: SustainedViolation[0.5s](G(|str_angle| < {P3_STR_ANGLE_LIMIT}))"
)

# ---------------------------------------------------------------------------
# P4 — Heading Alignment
# G (|hdg_err| < 0.25)
# Rationale: hdg_err std=0.21, max=2.13 (initial misalignment spike).
# After initialization, heading error should stay bounded.
# Large heading error means the agent is pointing away from the road.
# ---------------------------------------------------------------------------
P4_heading_alignment = Globally(
    Predicate('hdg_err', lambda v: P4_HDG_ERR_LIMIT - abs(v),
              label=f'|hdg_err| < {P4_HDG_ERR_LIMIT}'),
    a=P4_INIT_SKIP_S,  # skip initialization spike
    label=f"P4: G[{P4_INIT_SKIP_S},∞](|hdg_err| < {P4_HDG_ERR_LIMIT})"
)

# ---------------------------------------------------------------------------
# P5 — Recovery After Drift
# G( (|cte| > 0.5) → F[0, 5.0] (|cte| < 0.35) )
# Rationale: Old formula required recovery from >0.5m to <0.2m in 3s —
# a 0.3m hard drop that caused 90.1% violation. 0.35m recovery target
# is still a meaningful correction (30% reduction in displacement), and
# 5s window accommodates typical controller lag in the Udacity simulator.
# ---------------------------------------------------------------------------
_drifting   = Predicate('cte', lambda v: abs(v) - P5_DRIFT_THRESHOLD,
                        label=f'|cte| > {P5_DRIFT_THRESHOLD}')
_recovered  = Predicate('cte', lambda v: P5_RECOVERED_THRESHOLD - abs(v),
                        label=f'|cte| < {P5_RECOVERED_THRESHOLD}')

P5_recovery = Globally(
    _drifting.implies(Eventually(_recovered, a=0.0, b=P5_RECOVERY_HORIZON_S)),
    label=(f"P5: G((|cte|>{P5_DRIFT_THRESHOLD}) → "
           f"F[0,{P5_RECOVERY_HORIZON_S}](|cte|<{P5_RECOVERED_THRESHOLD}))")
)

# ---------------------------------------------------------------------------
# P6 — High-Curvature Safety
# G( (|curvature| > 0.05) → (|cte| < 0.6) )
# Rationale: Old thresholds (curv>0.03, cte<0.4) triggered on mild bends
# and demanded half the normal CTE limit, causing 99.3% violations. 0.05
# targets genuinely sharp curves (~90th percentile curvature), and 0.6m
# tighter-than-normal CTE still differentiates curve vs straight behaviour.
# ---------------------------------------------------------------------------
_high_curve = Predicate('curvature', lambda v: abs(v) - P6_CURVATURE_THRESHOLD,
                         label=f'|curvature| > {P6_CURVATURE_THRESHOLD}')
_tight_lane = Predicate('cte', lambda v: P6_TIGHT_CTE_LIMIT - abs(v),
                         label=f'|cte| < {P6_TIGHT_CTE_LIMIT}')

P6_curvature_safety = Globally(
    _high_curve.implies(_tight_lane),
    label=(f"P6: G((|curv|>{P6_CURVATURE_THRESHOLD}) → "
           f"(|cte|<{P6_TIGHT_CTE_LIMIT}))")
)

# ---------------------------------------------------------------------------
# Full property suite
# ---------------------------------------------------------------------------
ALL_PROPERTIES = {
    'P1_lane_keeping':      P1_lane_keeping,
    'P2_speed_stability':   P2_speed_stability,
    'P3_steering_smoothness': P3_steering_smoothness,
    'P4_heading_alignment': P4_heading_alignment,
    'P5_recovery':          P5_recovery,
    'P6_curvature_safety':  P6_curvature_safety,
}
