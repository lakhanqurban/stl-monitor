# ADS-Signal Temporal Logic (STL)-Monitor

**Runtime Verification and Safety Assurance Framework for Autonomous Systems using**

This project is an offline STL robustness monitor over simulation
traces originally collected for [*Coverage-Guided Road Selection and
Prioritization for Efficient Testing in ADS*](https://arxiv.org/abs/2601.08609).
Using the same Udacity self-driving car simulator data with the
NVIDIA Dave-2 model as the system under test, it adds a formal
temporal-analysis layer on top of that empirical test campaign —
mapping scenario-based ADS testing outcomes to STL robustness and
bridging simulation-driven V&V with formal specification-based
evaluation.

This project extends the same campaign with a formal runtime-verification layer:

1. **Specification layer** — encode expected ADS behavior as STL
   formulas over measured signals (speed, CTE, steering, heading,
   curvature).
2. **Quantitative verdicts** — replace binary pass/fail with a
   robustness score `ρ`, so each scenario has a measurable
   satisfaction margin.
3. **Post-hoc formal analysis** — compute violation timing,
   violation duration, recovery behavior, and aggregate summaries
   over the full scenario corpus.

For a deeper walkthrough of how the code is structured, see
[`ARCHITECTURE.md`](./ARCHITECTURE.md).

---

## Two Monitoring Modes

### 1. **Offline Analysis** (Post-hoc)
Evaluate completed simulation traces from CSV files:
- Full corpus processing (100s–1000s of roads)
- Comprehensive violation timing and recovery analysis
- Aggregate statistical summaries and heatmaps
- Publication-ready reports

```bash
python runner.py --data_dir ./dynamic_data --recursive --output_dir ./results
```

### 2. **Real-Time Verification** (Live)
*(New)* Evaluate STL properties during simulation execution:
- Per-step robustness computation as the agent drives
- Live terminal alerts on property violations
- Auto-opening HTML dashboard with live updates
- Structured CSV logging (stream, alerts, episode summary)
- Outputs saved under `stl_monitor/{logs,real_time_results}`

```bash
# Terminal view only
python ../main.py --stl-live-view terminal

# HTML dashboard auto-opens in browser
python ../main.py --stl-live-view html

# Both terminal and browser (recommended)
python ../main.py --stl-live-view both
```

---

## Properties

The runner evaluates six STL properties and a set of derived metrics:

| Metric | Meaning |
|--------|---------|
| `P1_lane_keeping` | Lane-keeping property: `abs(cte) < 0.8` |
| `P2_speed_stability` | Speed property: `speed > 5` after a 2 s warm-up |
| `P3_steering_smoothness` | Steering property: `abs(str_angle) < 0.7` |
| `P4_heading_alignment` | Heading property: `abs(hdg_err) < 0.25` after 0.5 s |
| `P5_recovery` | Recovery property: if `abs(cte) > 0.5`, return to `abs(cte) < 0.2` within 3 s |
| `P6_curvature_safety` | Curvature property: if `abs(curv) > 0.03`, keep `abs(cte) < 0.4` |
| `max_abs_cte` | Worst absolute cross-track error on the road |
| `cte_boundary_violation_rate` | Fraction of samples with `abs(cte) >= 1.5` |
| `cte_spike_count_gt_1p0` | Number of contiguous `abs(cte) > 1.0` spikes |
| `cte_recovery_count_within_2p5s` | Number of spikes that recover within 2.5 s |
| `mean_abs_cte_on_high_curvature` | Mean absolute CTE on high-curvature samples (`abs(curvature) > 0.03`) |
| `max_abs_cte_on_high_curvature` | Maximum absolute CTE on high-curvature samples (`abs(curvature) > 0.03`) |
| `steering_jerk_rate_0p1` | Fraction of steering changes with `abs(Δsteering) >= 0.1` |
| `mean_abs_steering_delta` | Mean absolute steering change between samples |
| `throttle_jerk_rate_0p2` | Fraction of throttle changes with `abs(Δthrottle) >= 0.2`, if throttle exists |

---

## STL semantics (brief)

Each property is evaluated as a robustness score `ρ` — a single real
number summarizing how strongly the property is satisfied on a given
trace. The sign is the verdict:

- `ρ > 0` → satisfied (the larger, the more "slack")
- `ρ < 0` → violated (the more negative, the worse)
- `ρ = 0` → right on the boundary

The formulas in our property suite are built from three small
building blocks: a single check against a threshold (e.g.
`|cte| < 0.8`), the usual boolean connectives (`AND`, `OR`, `NOT`,
`implies`), and the temporal operators `G` (must hold throughout a
time window) and `F` (must hold at least once in a time window).
They compose in the usual way — for example, P5's "if we drift, we
must recover within 3 s" is written as
`G( drifting → F[0,3](recovered) )` — and the implementation in
[`stl_monitor.py`](./stl_monitor.py) walks the resulting formula
tree once to produce the full robustness number in a single call.

The full quantitative semantics follow Donzé & Maler (2010); for
the formal equations and the per-class rule mapping, see
[`ARCHITECTURE.md`](./ARCHITECTURE.md#stl-semantics-full-reference).

---

## Installation & quick start

### Prerequisites

```bash
pip install pandas numpy matplotlib
```

### Offline Analysis (Historical Traces)

Evaluate a folder of scenario CSVs:

```bash
# Recursively evaluate scenario folders (e.g. dynamic_data/a*/**/*.csv)
python runner.py --data_dir ./dynamic_data --recursive --output_dir ./results
```

### Real-Time Monitoring (Live Simulation)

Run the main simulation with STL live verification:

```bash
# From parent directory
cd ..

# Run with terminal STL output
python main.py --num-episodes 10 --stl-live-view terminal

# Run with auto-opening HTML dashboard
python main.py --num-episodes 10 --stl-live-view html

# Run with both (recommended for full observability)
python main.py --num-episodes 10 --stl-live-view both

```

#### Real-Time CLI Flags

| Flag | Values | Default | Purpose |
|------|--------|---------|----------|
| `--stl-live-view` | `terminal` \| `html` \| `both` | `both` | How to display real-time STL results |
| `--stl-print-every` | integer | `25` | Print terminal summary every N steps |
| `--stl-html-refresh-steps` | integer | `25` | Refresh HTML dashboard every N steps |
| `--stl-alert-threshold` | float | `0.0` | Robustness threshold for violation alerts (ρ < threshold) |
| `--stl-no-auto-open-dashboard` | flag | not set | Do not auto-open HTML dashboard in browser |
| `--disable-real-time-stl` | flag | not set | Disable real-time STL monitoring entirely |

The implementation has no external STL dependency — the semantics are
self-contained in [`stl_monitor.py`](./stl_monitor.py).

---

## Real-Time Monitoring Demo

![Real-time STL dashboard showing live property evaluation during ADS simulation](./real_time_results/demo_realtime_stl.gif)

*Real-time STL verification with live terminal alerts and auto-refreshing HTML dashboard. The dashboard displays per-step robustness scores (ρ) and violation status for all six STL properties as the agent drives.*

---

## Outputs
### Offline Analysis Outputs
| File | Format | Content |
|------|--------|---------|
| `results.csv` | one row per road | per-property `*_rho` and `*_ok`, plus all derived metrics |
| `violations_detailed.csv` | one row per violation segment | `road_id`, `property`, `segment_id`, `start_time_s`, `end_time_s`, `duration_s` |
| `violations_summary.csv` | one row per (road, property) | `violated`, `violation_segments`, `total_violation_duration_s` |
| `analysis_metrics.csv` | one row per road | the full set of analysis metrics |
| `summary_overall_clean.csv` | one row | global summary (roads evaluated, p95 risk, top-risk road, weighted risk score) |
| `summary_properties_clean.csv` | one row per property | violation rates, duration stats, finite-ρ stats, `nan`/`inf` counts |
| `summary_metrics_clean.csv` | one row per analysis metric | mean / median / p95 / max / winsorized mean |
| `summary_top_risk_roads.csv` | top 20 roads | ranked by weighted risk score |
| `heatmap.png` | plot | per-road robustness heatmap for all six STL properties |
| `violation_rates.png` | plot | bar chart of per-property violation rate |
| `analysis_metrics_heatmap.png` | plot | normalized heatmap of the analysis metrics across roads |
| `trace_road_<id>.png` | plot (optional) | per-time-step robustness trace for a selected road |

### Real-Time Monitoring Outputs

*Saved under `stl_monitor/real_time_results/` and `stl_monitor/logs/`:*

| File | Format | Content |
|------|--------|----------|
| `realtime_stream.csv` | append-only | per-step signal values and robustness scores |
| `realtime_alerts.csv` | append-only | violation transitions (when ρ crosses 0) |
| `realtime_episode_summary.csv` | append-only | per-episode verdict and max violation counts |
| `realtime_dashboard.html` | live HTML | auto-refreshing browser dashboard with plots |

`results.csv` and the summary CSVs are the main artifacts for
offline analysis. The real-time CSV streams and HTML dashboard provide
live insights during simulation; the PNGs are generated automatically for visual inspection.

---

## Major results (full run, `Dave-2/ambiegen_2`, 973 roads)

The following headline numbers are from the latest full run stored
in `./results`.

### Overall

| Metric | Value |
|--------|-------|
| Roads evaluated | 973 |
| Avg total violation duration per road | 9.265 s |
| P95 total violation duration per road | 18.233 s |
| Top-risk road ID | 349 |
| Top-risk total violation duration | 48.830 s |

### STL property outcomes

| Property | Violated roads (%) | Mean violation duration (s) | P95 violation duration (s) |
|----------|---------------------|-----------------------------|----------------------------|
| P1 lane keeping | 87.87 | 1.402 | 3.457 |
| P2 speed stability | 0.10 | 0.000 | 0.000 |
| P3 steering smoothness | 50.15 | 0.086 | 0.408 |
| P4 heading alignment | 14.08 | 0.020 | 0.108 |
| P5 recovery | 90.13 | 2.263 | 5.679 |
| P6 curvature safety | 99.28 | 5.493 | 10.098 |

### Key interpretable metric summaries

| Metric | Mean | Median | P95 |
|--------|------|--------|-----|
| `mean_abs_cte` | 0.336 | 0.333 | 0.463 |
| `cte_near_boundary_rate` | 0.0238 | 0.0088 | 0.0933 |
| `steering_jerk_rate_0p1` | 0.0085 | 0.0074 | 0.0208 |
| `cte_recovery_success_rate_2p5s` | 0.5338 | 0.5000 | 1.0000 |

---

## Citation

The simulation traces used here were originally collected for:

> *Coverage-Guided Road Selection and Prioritization for Efficient
> Testing in ADS.* arXiv:2601.08609.
> [https://arxiv.org/abs/2601.08609](https://arxiv.org/abs/2601.08609)

The STL robustness layer in this repository follows:

> A. Donzé and O. Maler, "Robust Satisfaction of Temporal Logic over
> Real-Valued Signals," *Formal Modeling and Analysis of Timed
> Systems*, 2010.
