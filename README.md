# ADS-STL-Monitor

**Runtime verification of Autonomous Driving agent behavior using Signal Temporal Logic (STL)**

This project implements an offline STL robustness monitor over simulation traces
that were originally collected for the study *Coverage-Guided Road Selection and
Prioritization for Efficient Testing in ADS* ([arXiv:2601.08609](https://arxiv.org/abs/2601.08609)).
Using the same Udacity self-driving car simulator data collected using NVIDIA Dave-2 model as SUT, this repository adds a
formal temporal-analysis layer on top of that empirical test campaign. It maps
scenario-based ADS testing outcomes to Signal Temporal Logic (STL) robustness,
bridging simulation-driven V&V and formal specification-based evaluation.

---

## Motivation

The source traces in this repository were generated for the road-prioritization
study [arXiv:2601.08609](https://arxiv.org/abs/2601.08609), where the primary
goal was efficient scenario selection and fault-revealing test execution.

This project extends that same campaign with a formal runtime-verification layer:

1. **Specification layer**: encode expected ADS behavior as STL formulas over
   measured signals (speed, CTE, steering, heading, curvature).
2. **Quantitative verdicts**: replace binary pass/fail with robustness (ρ), so
   each scenario has a measurable satisfaction margin.
3. **Post-hoc formal analysis**: compute violation timing, violation duration,
   recovery behavior, and aggregate summaries over the full scenario corpus.

In short, the prior work provides the scenario-generation and prioritization
pipeline; this repository provides a formal temporal-analysis layer on top of
those generated traces.

---

## Runner Metrics

The runner evaluates STL properties and exports post-hoc metrics that are used
in the CSV summaries and plots.

| Metric | Meaning |
|--------|---------|
| `P1_lane_keeping` | Lane-keeping property: `abs(cte) < 0.8` |
| `P2_speed_stability` | Speed property: `speed > 5` after a 2 s warm-up |
| `P3_steering_smoothness` | Steering property: `abs(str_angle) < 0.7` |
| `P4_heading_alignment` | Heading property: `abs(hdg_err) < 0.25` after 0.5 s |
| `P5_recovery` | Recovery property: if `abs(cte) > 0.5`, return to `abs(cte) < 0.2` within 3 s |
| `P6_curvature_safety` | Curvature property: if `abs(curv) > 0.03`, keep `abs(cte) < 0.4` |
| `max_abs_cte` | Worst absolute cross-track error on the road |
| `mean_abs_cte` | Average absolute cross-track error |
| `cte_boundary_violation_rate` | Fraction of samples with `abs(cte) >= 1.5` |
| `cte_near_boundary_rate` | Fraction of samples with `abs(cte) >= 1.0` |
| `cte_spike_count_gt_1p0` | Number of contiguous `abs(cte) > 1.0` spikes |
| `cte_recovery_count_within_2p5s` | Number of spikes that recover within 2.5 s |
| `cte_recovery_success_rate_2p5s` | Fraction of spikes that recover within 2.5 s |
| `cte_recovery_latency_mean_s` | Mean recovery latency for recovered spikes |
| `cte_recovery_latency_max_s` | Maximum recovery latency for recovered spikes |
| `steering_jerk_rate_0p1` | Fraction of steering changes with `abs(Δsteering) >= 0.1` |
| `mean_abs_steering_delta` | Mean absolute steering change between samples |
| `speed_over_threshold_rate` | Fraction of samples with `speed >= 30` |
| `mean_abs_throttle_delta` | Mean absolute throttle change, if throttle exists |
| `throttle_jerk_rate_0p2` | Fraction of throttle changes with `abs(Δthrottle) >= 0.2`, if throttle exists |

---

## STL Semantics

Robustness is computed using standard quantitative STL semantics
(Donzé & Maler, 2010):

```
ρ(f(x) ≥ 0, s, t)  =  f(s(t))
ρ(¬φ, s, t)         = -ρ(φ, s, t)
ρ(φ ∧ ψ, s, t)      =  min(ρ(φ,s,t), ρ(ψ,s,t))
ρ(G[a,b]φ, s, t)    =  min_{t'∈[t+a,t+b]} ρ(φ, s, t')
ρ(F[a,b]φ, s, t)    =  max_{t'∈[t+a,t+b]} ρ(φ, s, t')
```

Implemented from scratch in `stl_monitor.py` without external STL libraries,
making the semantics explicit and inspectable.

---

## Usage

```bash

# Recursively evaluate scenario folders (e.g., dynamic_data/a*/**/*.csv)
python runner.py --data_dir ./dynamic_data --recursive --output_dir ./results
```

### Output: `results.csv`

| road_id | P1_lane_keeping_rho | P1_lane_keeping_ok | ... |
|---------|---------------------|--------------------|-----|
| 0       | -0.289              | False              | ... |
| 1       | +0.412              | True               | ... |

### Output: `violations_detailed.csv`

One row per violation segment, explicitly reporting **where** and **for how long**
each property is violated.

| road_id | property | segment_id | start_time_s | end_time_s | duration_s |
|---------|----------|------------|--------------|------------|------------|
| 0       | P1_lane_keeping | 1 | 10.70 | 11.50 | 0.80 |

### Output: `violations_summary.csv`

Per road/property summary used for scenario-property violation matrices.

| road_id | property | violated | violation_segments | total_violation_duration_s |
|---------|----------|----------|--------------------|----------------------------|
| 0       | P1_lane_keeping | True | 1 | 0.80 |

### Output: `analysis_metrics.csv`

The CSV contains the metrics listed above for each road.

### Output: `analysis_metrics_heatmap.png`

Normalized visualization of the new CSV-derived metrics across roads. This makes it easy to spot
which scenarios are most extreme in lane tracking, recovery latency, and steering smoothness.

### Output: Clean Summary CSVs

Publication-ready aggregate reports are exported automatically:

| File | Purpose |
|------|---------|
| `summary_overall_clean.csv` | One-row global summary (roads evaluated, p95 risk, top-risk road) |
| `summary_properties_clean.csv` | Per-property violation rates, duration stats, finite-ρ stats, `inf` counts |
| `summary_metrics_clean.csv` | Distribution summary for each analysis metric (mean/median/p95/max/winsorized mean) |
| `summary_top_risk_roads.csv` | Top 20 roads by total violation duration |

### Major Results (Full Run, `Dave-2/a2`, 973 roads)

The following headline numbers are from the latest full run stored in `./results`.

#### Overall

| Metric | Value |
|--------|-------|
| Roads evaluated | 973 |
| Avg total violation duration per road | 9.265 s |
| P95 total violation duration per road | 18.233 s |
| Top-risk road ID | 349 |
| Top-risk total violation duration | 48.830 s |

#### STL Property Outcomes

| Property | Violated roads (%) | Mean violation duration (s) | P95 violation duration (s) |
|----------|---------------------|-----------------------------|----------------------------|
| P1 lane keeping | 87.87 | 1.402 | 3.457 |
| P2 speed stability | 0.10 | 0.000 | 0.000 |
| P3 steering smoothness | 50.15 | 0.086 | 0.408 |
| P4 heading alignment | 14.08 | 0.020 | 0.108 |
| P5 recovery | 90.13 | 2.263 | 5.679 |
| P6 curvature safety | 99.28 | 5.493 | 10.098 |

#### Key Interpretable Metric Summaries

| Metric | Mean | Median | P95 |
|--------|------|--------|-----|
| `mean_abs_cte` | 0.336 | 0.333 | 0.463 |
| `cte_near_boundary_rate` | 0.0238 | 0.0088 | 0.0933 |
| `steering_jerk_rate_0p1` | 0.0085 | 0.0074 | 0.0208 |
| `cte_recovery_success_rate_2p5s` | 0.5338 | 0.5000 | 1.0000 |

Notes:
- `P2_speed_stability_rho` and `P4_heading_alignment_rho` may include `+inf` on short traces with empty temporal windows; clean summaries report finite-ρ statistics and `inf` counts explicitly.
- `max_abs_cte` has a strong outlier (61.287); use `winsorized_mean_1_99` in `summary_metrics_clean.csv` for robust aggregate reporting.

---

## Dependencies

```
pip install pandas numpy matplotlib
```

No external STL library required: semantics are self-contained in
`stl_monitor.py`.

---
