"""
ADS STL Property Suite
======================
Six properties grounded in the actual signal ranges observed in the
Udacity ADS simulation data. Each property is:

  P1 — Lane Keeping Safety:   |CTE| must always stay below 0.8m
  P2 — Speed Stability:       Speed must stay above 5 mph once moving
  P3 — Steering Smoothness:   Steering angle changes must be gradual
  P4 — Heading Alignment:     Heading error must stay bounded
  P5 — Recovery After Drift:  After large CTE, agent must recover
  P6 — High-Curvature Safety: On sharp curves, CTE must be tighter

Each property is represented as an STLFormula object.
Thresholds are derived from data statistics (see analysis in README).
"""

from stl_monitor import (
    Signal, Predicate, Not, And, Or, Globally, Eventually
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
P1_CTE_LIMIT = 0.8

# P2 — Speed Stability
P2_WARMUP_S = 2.0
P2_MIN_SPEED = 5.0

# P3 — Steering Smoothness
P3_STR_ANGLE_LIMIT = 0.7

# P4 — Heading Alignment
P4_INIT_SKIP_S = 0.5
P4_HDG_ERR_LIMIT = 0.25

# P5 — Recovery After Drift
P5_DRIFT_THRESHOLD = 0.5
P5_RECOVERED_THRESHOLD = 0.2
P5_RECOVERY_HORIZON_S = 3.0

# P6 — High-Curvature Safety
P6_CURVATURE_THRESHOLD = 0.03
P6_TIGHT_CTE_LIMIT = 0.4


# ---------------------------------------------------------------------------
# P1 — Lane Keeping Safety
# G (|cte| < 0.8)
# Rationale: CTE std=0.36, max=1.09. 0.8 = ~2σ safety threshold.
# Violation means the agent is dangerously close to leaving its lane.
# ---------------------------------------------------------------------------
P1_lane_keeping = Globally(
    Predicate('cte', lambda v: P1_CTE_LIMIT - abs(v),
              label=f'|cte| < {P1_CTE_LIMIT}'),
    label=f"P1: G(|cte| < {P1_CTE_LIMIT})"
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
# G (|str_angle| < 0.7)
# Rationale: str_angle std=0.23, max=0.83. Values above 0.7 indicate
# near-maximum steering — a proxy for loss of smooth control.
# Abrupt steering is a precursor to instability.
# ---------------------------------------------------------------------------
P3_steering_smoothness = Globally(
    Predicate('str_angle', lambda v: P3_STR_ANGLE_LIMIT - abs(v),
              label=f'|str_angle| < {P3_STR_ANGLE_LIMIT}'),
    label=f"P3: G(|str_angle| < {P3_STR_ANGLE_LIMIT})"
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
# G( (|cte| > 0.5) → F[0, 3.0] (|cte| < 0.2) )
# Rationale: If the agent drifts significantly (|CTE| > 0.5),
# it must recover within 3 seconds. Tests the agent's self-correction
# capability — key for resilience evaluation.
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
# G( (|curvature| > 0.03) → (|cte| < 0.4) )
# Rationale: On sharp curves (|curvature| > 0.03, ~75th percentile),
# lane keeping must be tighter (0.4m vs 0.8m normal threshold).
# This is a context-dependent safety property — more demanding than P1.
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
