"""
STL Monitor — Offline Signal Temporal Logic robustness evaluation
for ADS simulation signals.

Implements the standard (qualitative + quantitative) STL semantics:
  - Atomic predicates over real-valued signals
  - Boolean connectives: NOT, AND, OR, IMPLIES
  - Temporal operators: G (globally), F (eventually), U (until)
  - Bounded temporal operators: G[a,b], F[a,b]

Robustness (rho): positive = property satisfied with margin,
                  negative = property violated, zero = boundary.

Reference: Donzé & Maler (2010), "Robust Satisfaction of Temporal
Logic over Real-Valued Signals"
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional


# ---------------------------------------------------------------------------
# Signal representation
# ---------------------------------------------------------------------------

@dataclass
class Signal:
    """
    A real-valued signal sampled at discrete timestamps.
    times: 1-D array of timestamps (must be monotonically increasing)
    values: 1-D array of signal values at each timestamp
    """
    times: np.ndarray
    values: np.ndarray

    def __post_init__(self):
        self.times = np.asarray(self.times, dtype=float)
        self.values = np.asarray(self.values, dtype=float)
        assert len(self.times) == len(self.values), "times/values length mismatch"

    def at(self, t: float) -> float:
        """Linear interpolation at time t."""
        return float(np.interp(t, self.times, self.values))


# ---------------------------------------------------------------------------
# STL formula nodes
# ---------------------------------------------------------------------------

class STLFormula:
    """Base class for all STL formula nodes."""

    def robustness(self, signals: dict, t: float) -> float:
        raise NotImplementedError

    def robustness_trace(self, signals: dict) -> np.ndarray:
        """Compute robustness at every timestamp in the first signal."""
        times = next(iter(signals.values())).times
        return np.array([self.robustness(signals, t) for t in times])

    def satisfies(self, signals: dict, t: float = 0.0) -> bool:
        return self.robustness(signals, t) >= 0

    # Operator sugar
    def __and__(self, other): return And(self, other)
    def __or__(self, other):  return Or(self, other)
    def __invert__(self):     return Not(self)
    def implies(self, other): return Or(Not(self), other)


class Predicate(STLFormula):
    """
    Atomic predicate: f(signal_name) - threshold >= 0
    e.g. Predicate('speed', lambda v: 30 - v)  → speed <= 30
         Predicate('cte',   lambda v: 1.5 - abs(v))  → |cte| <= 1.5
    """
    def __init__(self, signal_name: str, fn: Callable[[float], float], label: str = ""):
        self.signal_name = signal_name
        self.fn = fn
        self.label = label or f"pred({signal_name})"

    def robustness(self, signals: dict, t: float) -> float:
        v = signals[self.signal_name].at(t)
        return float(self.fn(v))

    def __repr__(self):
        return self.label


class Not(STLFormula):
    def __init__(self, phi):
        self.phi = phi

    def robustness(self, signals, t):
        return -self.phi.robustness(signals, t)

    def __repr__(self):
        return f"¬({self.phi})"


class And(STLFormula):
    def __init__(self, phi, psi):
        self.phi, self.psi = phi, psi

    def robustness(self, signals, t):
        return min(self.phi.robustness(signals, t),
                   self.psi.robustness(signals, t))

    def __repr__(self):
        return f"({self.phi} ∧ {self.psi})"


class Or(STLFormula):
    def __init__(self, phi, psi):
        self.phi, self.psi = phi, psi

    def robustness(self, signals, t):
        return max(self.phi.robustness(signals, t),
                   self.psi.robustness(signals, t))

    def __repr__(self):
        return f"({self.phi} ∨ {self.psi})"


class Globally(STLFormula):
    """
    G[a,b] phi: phi holds at all times in [t+a, t+b].
    If b is None, evaluates over [t+a, end of signal].
    """
    def __init__(self, phi, a: float = 0.0, b: Optional[float] = None, label: str = ""):
        self.phi, self.a, self.b = phi, a, b
        self.label = label

    def robustness(self, signals, t):
        times = next(iter(signals.values())).times
        t_lo = t + self.a
        t_hi = (t + self.b) if self.b is not None else times[-1]
        window = times[(times >= t_lo) & (times <= t_hi)]
        if len(window) == 0:
            return float('inf')
        return float(min(self.phi.robustness(signals, tau) for tau in window))

    def __repr__(self):
        interval = f"[{self.a},{self.b}]" if self.b is not None else f"[{self.a},∞]"
        return f"G{interval}({self.phi})"


class Eventually(STLFormula):
    """
    F[a,b] phi: phi holds at some time in [t+a, t+b].
    """
    def __init__(self, phi, a=0.0, b=None, label=""):
        self.phi, self.a, self.b = phi, a, b
        self.label = label

    def robustness(self, signals, t):
        times = next(iter(signals.values())).times
        t_lo = t + self.a
        t_hi = (t + self.b) if self.b is not None else times[-1]
        window = times[(times >= t_lo) & (times <= t_hi)]
        if len(window) == 0:
            return float('-inf')
        return float(max(self.phi.robustness(signals, tau) for tau in window))

    def __repr__(self):
        interval = f"[{self.a},{self.b}]" if self.b is not None else f"[{self.a},∞]"
        return f"F{interval}({self.phi})"
