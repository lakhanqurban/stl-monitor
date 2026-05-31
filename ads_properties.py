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
# P1 — Lane Keeping Safety
# G (|cte| < 0.8)
# Rationale: CTE std=0.36, max=1.09. 0.8 = ~2σ safety threshold.
# Violation means the agent is dangerously close to leaving its lane.
# ---------------------------------------------------------------------------
P1_lane_keeping = Globally(
    Predicate('cte', lambda v: 0.8 - abs(v),
              label='|cte| < 0.8'),
    label="P1: G(|cte| < 0.8)"
)

# ---------------------------------------------------------------------------
# P2 — Speed Stability (after warm-up)
# G[2.0, ∞] (speed > 5.0)
# Rationale: Agent starts from rest. After 2s warm-up it should be moving.
# Mean speed = 27.9 mph; min meaningful speed = 5 mph.
# Stalling mid-road is a safety failure mode.
# ---------------------------------------------------------------------------
P2_speed_stability = Globally(
    Predicate('speed', lambda v: v - 5.0,
              label='speed > 5.0'),
    a=2.0,
    label="P2: G[2,∞](speed > 5)"
)

# ---------------------------------------------------------------------------
# P3 — Steering Smoothness
# G (|str_angle| < 0.7)
# Rationale: str_angle std=0.23, max=0.83. Values above 0.7 indicate
# near-maximum steering — a proxy for loss of smooth control.
# Abrupt steering is a precursor to instability.
# ---------------------------------------------------------------------------
P3_steering_smoothness = Globally(
    Predicate('str_angle', lambda v: 0.7 - abs(v),
              label='|str_angle| < 0.7'),
    label="P3: G(|str_angle| < 0.7)"
)

# ---------------------------------------------------------------------------
# P4 — Heading Alignment
# G (|hdg_err| < 0.25)
# Rationale: hdg_err std=0.21, max=2.13 (initial misalignment spike).
# After initialization, heading error should stay bounded.
# Large heading error means the agent is pointing away from the road.
# ---------------------------------------------------------------------------
P4_heading_alignment = Globally(
    Predicate('hdg_err', lambda v: 0.25 - abs(v),
              label='|hdg_err| < 0.25'),
    a=0.5,  # skip initialization spike
    label="P4: G[0.5,∞](|hdg_err| < 0.25)"
)

# ---------------------------------------------------------------------------
# P5 — Recovery After Drift
# G( (|cte| > 0.5) → F[0, 3.0] (|cte| < 0.2) )
# Rationale: If the agent drifts significantly (|CTE| > 0.5),
# it must recover within 3 seconds. Tests the agent's self-correction
# capability — key for resilience evaluation.
# ---------------------------------------------------------------------------
_drifting   = Predicate('cte', lambda v: abs(v) - 0.5, label='|cte| > 0.5')
_recovered  = Predicate('cte', lambda v: 0.2 - abs(v),  label='|cte| < 0.2')

P5_recovery = Globally(
    _drifting.implies(Eventually(_recovered, a=0.0, b=3.0)),
    label="P5: G((|cte|>0.5) → F[0,3](|cte|<0.2))"
)

# ---------------------------------------------------------------------------
# P6 — High-Curvature Safety
# G( (|curvature| > 0.03) → (|cte| < 0.4) )
# Rationale: On sharp curves (|curvature| > 0.03, ~75th percentile),
# lane keeping must be tighter (0.4m vs 0.8m normal threshold).
# This is a context-dependent safety property — more demanding than P1.
# ---------------------------------------------------------------------------
_high_curve = Predicate('curvature', lambda v: abs(v) - 0.03,
                         label='|curvature| > 0.03')
_tight_lane = Predicate('cte', lambda v: 0.4 - abs(v),
                         label='|cte| < 0.4')

P6_curvature_safety = Globally(
    _high_curve.implies(_tight_lane),
    label="P6: G((|curv|>0.03) → (|cte|<0.4))"
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
