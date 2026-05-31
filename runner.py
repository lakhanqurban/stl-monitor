"""
STL Monitor Runner
==================
Loads ADS simulation CSVs (one per road scenario), evaluates all STL
properties, and produces:
  1. results.csv  — per-road robustness scores for each property
  2. violation heatmap (PNG)
    3. clean summary CSVs for publication-ready reporting
    4. analysis metrics heatmap (PNG)
    5. robustness trace plots for selected roads (PNG)

Usage:
    python runner.py --data_dir ./data --output_dir ./results
    python runner.py --data_dir ./data --output_dir ./results --trace_roads 0 5
"""

import os, sys, argparse
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from stl_monitor import Signal
from ads_properties import ALL_PROPERTIES

SIGNAL_COLS = ['speed', 'cte', 'str_angle', 'hdg_err', 'curvature']

CTE_LANE_BOUNDARY = 1.5
CTE_NEAR_BOUNDARY = 1.0
STEERING_JERK_THRESHOLD = 0.10
THROTTLE_JERK_THRESHOLD = 0.20
CTE_RECOVERY_SPIKE_THRESHOLD = 1.0
CTE_RECOVERY_THRESHOLD = 0.3
CTE_RECOVERY_HORIZON_S = 2.5


def find_csv_files(data_dir, recursive=False):
    data_path = Path(data_dir)

    # Support passing a single CSV file path directly.
    if data_path.is_file() and data_path.suffix.lower() == '.csv':
        return [data_path]

    pattern = "**/*.csv" if recursive else "*.csv"
    csv_files = sorted(
        data_path.glob(pattern),
        key=lambda p: int(p.stem) if p.stem.isdigit() else p.stem,
    )
    return [p for p in csv_files if p.is_file()]

def load_signals(csv_path):
    df = pd.read_csv(csv_path).sort_values('timestamp').reset_index(drop=True)
    times = df['timestamp'].values
    signals = {col: Signal(times, df[col].values)
               for col in SIGNAL_COLS if col in df.columns}
    return signals, df

def evaluate_road(signals):
    results = {}
    for name, formula in ALL_PROPERTIES.items():
        try:
            rho = formula.robustness(signals, t=0.0)
            results[name] = {'robustness': round(float(rho), 4), 'satisfied': rho >= 0}
        except Exception as e:
            results[name] = {'robustness': float('nan'), 'satisfied': False}
    return results


def _mean_consecutive_abs_delta(values):
    if len(values) < 2:
        return float('nan')
    return float(np.mean(np.abs(np.diff(values))))


def _count_contiguous_true_segments(mask):
    if len(mask) == 0:
        return 0
    count = 0
    in_segment = False
    for is_true in mask:
        if is_true and not in_segment:
            count += 1
            in_segment = True
        elif not is_true:
            in_segment = False
    return count


def _recovery_latencies(times, cte_values, spike_threshold=CTE_RECOVERY_SPIKE_THRESHOLD,
                        recovery_threshold=CTE_RECOVERY_THRESHOLD,
                        horizon_s=CTE_RECOVERY_HORIZON_S):
    """Return recovery latencies in seconds for each CTE spike that recovers within the horizon."""
    latencies = []
    spike_indices = np.where(np.abs(cte_values) > spike_threshold)[0]
    used_spikes = set()
    for idx in spike_indices:
        if idx in used_spikes:
            continue
        t0 = times[idx]
        window = (times >= t0) & (times <= t0 + horizon_s)
        future_indices = np.where(window)[0]
        recover_indices = future_indices[np.where(np.abs(cte_values[window]) < recovery_threshold)[0]]
        if len(recover_indices) > 0:
            recovery_idx = recover_indices[0]
            latencies.append(float(times[recovery_idx] - t0))
            used_spikes.update(future_indices[future_indices <= recovery_idx])
    return latencies


def compute_analysis_metrics(df):
    """Compute additional CSV-derived metrics for safety, recovery, and comfort analysis."""
    metrics = {}
    times = df['timestamp'].values
    cte = df['cte'].values
    speed = df['speed'].values
    steering = df['str_angle'].values

    metrics['max_abs_cte'] = float(np.max(np.abs(cte))) if len(cte) else float('nan')
    metrics['mean_abs_cte'] = float(np.mean(np.abs(cte))) if len(cte) else float('nan')
    metrics['cte_boundary_violation_rate'] = float(np.mean(np.abs(cte) >= CTE_LANE_BOUNDARY)) if len(cte) else float('nan')
    metrics['cte_near_boundary_rate'] = float(np.mean(np.abs(cte) >= CTE_NEAR_BOUNDARY)) if len(cte) else float('nan')
    metrics['speed_over_threshold_rate'] = float(np.mean(speed >= 30.0)) if len(speed) else float('nan')
    metrics['steering_jerk_rate_0p1'] = float(np.mean(np.abs(np.diff(steering)) >= STEERING_JERK_THRESHOLD)) if len(steering) > 1 else float('nan')
    metrics['mean_abs_steering_delta'] = _mean_consecutive_abs_delta(steering)
    metrics['cte_spike_count_gt_1p0'] = int(_count_contiguous_true_segments(np.abs(cte) > CTE_RECOVERY_SPIKE_THRESHOLD))

    recovery_latencies = _recovery_latencies(times, cte)
    spike_count = metrics['cte_spike_count_gt_1p0']
    metrics['cte_recovery_count_within_2p5s'] = int(len(recovery_latencies))
    metrics['cte_recovery_success_rate_2p5s'] = float(len(recovery_latencies) / spike_count) if spike_count else float('nan')
    metrics['cte_recovery_latency_mean_s'] = float(np.mean(recovery_latencies)) if recovery_latencies else float('nan')
    metrics['cte_recovery_latency_max_s'] = float(np.max(recovery_latencies)) if recovery_latencies else float('nan')

    if 'throttle' in df.columns:
        throttle = df['throttle'].values
        metrics['mean_abs_throttle_delta'] = _mean_consecutive_abs_delta(throttle)
        metrics['throttle_jerk_rate_0p2'] = float(np.mean(np.abs(np.diff(throttle)) >= THROTTLE_JERK_THRESHOLD)) if len(throttle) > 1 else float('nan')
    else:
        metrics['mean_abs_throttle_delta'] = float('nan')
        metrics['throttle_jerk_rate_0p2'] = float('nan')

    return metrics


def _extract_violation_segments(times, violation_mask):
    """Return contiguous True segments as (start, end, duration)."""
    neg = np.asarray(violation_mask, dtype=bool)
    if len(neg) == 0:
        return []

    segments = []
    start_idx = None

    for i, is_neg in enumerate(neg):
        if is_neg and start_idx is None:
            start_idx = i
        elif not is_neg and start_idx is not None:
            end_idx = i - 1
            start_t = float(times[start_idx])
            end_t = float(times[end_idx])
            duration = max(0.0, end_t - start_t)
            segments.append((start_t, end_t, duration))
            start_idx = None

    if start_idx is not None:
        start_t = float(times[start_idx])
        end_t = float(times[-1])
        duration = max(0.0, end_t - start_t)
        segments.append((start_t, end_t, duration))

    return segments


def property_violation_mask(prop_name, df):
    """Return a boolean mask over timestamps where the property is violated."""
    times = df['timestamp'].values
    cte = df['cte'].values
    speed = df['speed'].values
    str_angle = df['str_angle'].values
    hdg_err = df['hdg_err'].values
    curvature = df['curvature'].values

    if prop_name == 'P1_lane_keeping':
        return np.abs(cte) >= 0.8

    if prop_name == 'P2_speed_stability':
        return (times >= 2.0) & (speed <= 5.0)

    if prop_name == 'P3_steering_smoothness':
        return np.abs(str_angle) >= 0.7

    if prop_name == 'P4_heading_alignment':
        return (times >= 0.5) & (np.abs(hdg_err) >= 0.25)

    if prop_name == 'P5_recovery':
        drifting = np.abs(cte) > 0.5
        recovered = np.abs(cte) < 0.2
        violated = np.zeros(len(df), dtype=bool)
        for i in np.where(drifting)[0]:
            t_end = times[i] + 3.0
            window = (times >= times[i]) & (times <= t_end)
            if not np.any(recovered[window]):
                violated[i] = True
        return violated

    if prop_name == 'P6_curvature_safety':
        return (np.abs(curvature) > 0.03) & (np.abs(cte) >= 0.4)

    return np.zeros(len(df), dtype=bool)


def build_violation_reports(csv_files):
    """
    Create detailed and summary violation reports across roads/properties:
      - detailed: each violated segment with start/end/duration
      - summary: counts and total violated duration per property and road
    """
    detailed_rows = []
    summary_rows = []

    for csv_path in csv_files:
        road_id = csv_path.stem
        try:
            _, df = load_signals(str(csv_path))
            times = df['timestamp'].values
        except Exception as e:
            print(f"  [WARN] Cannot analyze violations for {csv_path.name}: {e}")
            continue

        for prop_name in ALL_PROPERTIES:
            try:
                violation_mask = property_violation_mask(prop_name, df)
                segments = _extract_violation_segments(times, violation_mask)

                if segments:
                    for seg_idx, (start_t, end_t, duration) in enumerate(segments, start=1):
                        detailed_rows.append({
                            'road_id': road_id,
                            'property': prop_name,
                            'segment_id': seg_idx,
                            'start_time_s': round(start_t, 5),
                            'end_time_s': round(end_t, 5),
                            'duration_s': round(duration, 5),
                        })

                summary_rows.append({
                    'road_id': road_id,
                    'property': prop_name,
                    'violated': len(segments) > 0,
                    'violation_segments': len(segments),
                    'total_violation_duration_s': round(
                        float(sum(seg[2] for seg in segments)), 5
                    ),
                })
            except Exception as e:
                print(f"  [WARN] Violation analysis failed for {road_id}/{prop_name}: {e}")
                summary_rows.append({
                    'road_id': road_id,
                    'property': prop_name,
                    'violated': False,
                    'violation_segments': 0,
                    'total_violation_duration_s': np.nan,
                })

    detailed_df = pd.DataFrame(detailed_rows)
    summary_df = pd.DataFrame(summary_rows)
    return detailed_df, summary_df


def build_metrics_table(csv_files):
    rows = []
    for csv_path in csv_files:
        road_id = csv_path.stem
        try:
            _, df = load_signals(str(csv_path))
            row = {'road_id': road_id}
            row.update(compute_analysis_metrics(df))
            rows.append(row)
        except Exception as e:
            print(f"  [WARN] Metric computation failed for {csv_path.name}: {e}")
    return pd.DataFrame(rows)


def _winsorized_mean(series, lower_q=0.01, upper_q=0.99):
    s = pd.to_numeric(series, errors='coerce').dropna()
    if s.empty:
        return float('nan')
    lo = s.quantile(lower_q)
    hi = s.quantile(upper_q)
    return float(s.clip(lower=lo, upper=hi).mean())


def build_clean_summary_tables(results_df, violations_summary_df, metrics_df):
    property_rows = []
    for prop_name in ALL_PROPERTIES:
        prop_vs = violations_summary_df[violations_summary_df['property'] == prop_name]
        rho_col = f'{prop_name}_rho'
        rho = pd.to_numeric(results_df.get(rho_col, pd.Series(dtype=float)), errors='coerce')
        finite_rho = rho[np.isfinite(rho)]
        inf_count = int(np.isinf(rho).sum()) if len(rho) else 0

        property_rows.append({
            'property': prop_name,
            'roads': int(prop_vs['road_id'].nunique()),
            'violated_rate_pct': float(prop_vs['violated'].mean() * 100.0),
            'mean_segments': float(prop_vs['violation_segments'].mean()),
            'mean_violation_duration_s': float(prop_vs['total_violation_duration_s'].mean()),
            'p95_violation_duration_s': float(prop_vs['total_violation_duration_s'].quantile(0.95)),
            'mean_rho_finite': float(finite_rho.mean()) if not finite_rho.empty else float('nan'),
            'median_rho_finite': float(finite_rho.median()) if not finite_rho.empty else float('nan'),
            'p95_rho_finite': float(finite_rho.quantile(0.95)) if not finite_rho.empty else float('nan'),
            'inf_rho_count': inf_count,
        })

    property_summary_df = pd.DataFrame(property_rows)

    metric_rows = []
    for col in metrics_df.columns:
        if col == 'road_id':
            continue
        s = pd.to_numeric(metrics_df[col], errors='coerce').dropna()
        if s.empty:
            continue
        metric_rows.append({
            'metric': col,
            'count': int(len(s)),
            'mean': float(s.mean()),
            'median': float(s.median()),
            'p95': float(s.quantile(0.95)),
            'max': float(s.max()),
            'winsorized_mean_1_99': _winsorized_mean(s),
        })

    metrics_summary_df = pd.DataFrame(metric_rows)

    road_total = violations_summary_df.groupby('road_id', as_index=False).agg(
        total_violation_duration_s=('total_violation_duration_s', 'sum'),
        violated_properties_count=('violated', 'sum'),
    ).sort_values('total_violation_duration_s', ascending=False)

    overall_summary_df = pd.DataFrame([{
        'roads_evaluated': int(results_df['road_id'].nunique()),
        'properties_evaluated': int(len(ALL_PROPERTIES)),
        'avg_total_violation_duration_s_per_road': float(road_total['total_violation_duration_s'].mean()),
        'p95_total_violation_duration_s_per_road': float(road_total['total_violation_duration_s'].quantile(0.95)),
        'top_risk_road_id': str(road_total.iloc[0]['road_id']) if not road_total.empty else '',
        'top_risk_total_violation_duration_s': float(road_total.iloc[0]['total_violation_duration_s']) if not road_total.empty else float('nan'),
    }])

    top_risk_roads_df = road_total.head(20).reset_index(drop=True)

    return property_summary_df, metrics_summary_df, overall_summary_df, top_risk_roads_df


def write_clean_summary_csvs(output_dir, results_df, violations_summary_df, metrics_df):
    property_summary_df, metrics_summary_df, overall_summary_df, top_risk_roads_df = build_clean_summary_tables(
        results_df, violations_summary_df, metrics_df
    )

    property_summary_path = os.path.join(output_dir, 'summary_properties_clean.csv')
    metrics_summary_path = os.path.join(output_dir, 'summary_metrics_clean.csv')
    overall_summary_path = os.path.join(output_dir, 'summary_overall_clean.csv')
    top_risk_roads_path = os.path.join(output_dir, 'summary_top_risk_roads.csv')

    property_summary_df.to_csv(property_summary_path, index=False)
    metrics_summary_df.to_csv(metrics_summary_path, index=False)
    overall_summary_df.to_csv(overall_summary_path, index=False)
    top_risk_roads_df.to_csv(top_risk_roads_path, index=False)

    return {
        'property_summary_path': property_summary_path,
        'metrics_summary_path': metrics_summary_path,
        'overall_summary_path': overall_summary_path,
        'top_risk_roads_path': top_risk_roads_path,
    }

def plot_violation_heatmap(df, output_path):
    prop_names  = list(ALL_PROPERTIES.keys())
    rho_cols    = [f'{p}_rho' for p in prop_names]
    short_labels = [p.split('_',1)[1].replace('_',' ') for p in prop_names]
    rho_matrix  = np.clip(df[rho_cols].values.astype(float), -1.5, 1.5)
    n_roads     = len(df)
    fig, ax = plt.subplots(figsize=(10, min(max(6, n_roads*0.15), 20)))
    cmap = mcolors.LinearSegmentedColormap.from_list(
        'stl', ['#d73027','#fee08b','#1a9850'], N=256)
    im = ax.imshow(rho_matrix, aspect='auto', cmap=cmap, vmin=-1.5, vmax=1.5)
    ax.set_xticks(range(len(short_labels)))
    ax.set_xticklabels(short_labels, rotation=30, ha='right', fontsize=9)
    ax.set_xlabel('STL Property', fontsize=11)
    ax.set_ylabel('Road ID', fontsize=11)
    ax.set_title('STL Robustness Heatmap — ADS Simulation Scenarios\n'
                 '(green=satisfied, red=violated)', fontsize=12)
    step = max(1, n_roads // 20)
    ticks = list(range(0, n_roads, step))
    ax.set_yticks(ticks)
    ax.set_yticklabels(df['road_id'].iloc[ticks].tolist(), fontsize=7)
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Robustness ρ (clipped ±1.5)', fontsize=9)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved heatmap → {output_path}")

def plot_summary_bar(df, output_path):
    prop_names   = list(ALL_PROPERTIES.keys())
    ok_cols      = [f'{p}_ok' for p in prop_names]
    short_labels = [p.split('_',1)[1].replace('_',' ') for p in prop_names]
    vrates       = [(1 - df[c].mean())*100 for c in ok_cols]
    fig, ax = plt.subplots(figsize=(9,5))
    colors = ['#d73027' if v>20 else '#fee08b' if v>5 else '#1a9850' for v in vrates]
    bars = ax.bar(short_labels, vrates, color=colors)
    ax.set_ylabel('Violation Rate (%)', fontsize=11)
    ax.set_xlabel('STL Property', fontsize=11)
    ax.set_title('Property Violation Rate Across All Road Scenarios', fontsize=12)
    ax.set_ylim(0, 115)
    for bar, v in zip(bars, vrates):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                f'{v:.1f}%', ha='center', va='bottom', fontsize=9)
    plt.xticks(rotation=25, ha='right')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved summary bar → {output_path}")


def plot_analysis_metrics_heatmap(metrics_df, output_path):
    metric_cols = [
        'max_abs_cte',
        'mean_abs_cte',
        'cte_boundary_violation_rate',
        'cte_near_boundary_rate',
        'cte_spike_count_gt_1p0',
        'cte_recovery_count_within_2p5s',
        'cte_recovery_success_rate_2p5s',
        'cte_recovery_latency_mean_s',
        'steering_jerk_rate_0p1',
        'mean_abs_steering_delta',
        'speed_over_threshold_rate',
        'mean_abs_throttle_delta',
        'throttle_jerk_rate_0p2',
    ]

    available_cols = [c for c in metric_cols if c in metrics_df.columns]
    if not available_cols:
        print("  [WARN] No analysis metric columns available for heatmap.")
        return

    plot_df = metrics_df[['road_id'] + available_cols].copy()
    plot_df = plot_df.sort_values(by=available_cols[0], ascending=False)
    values = plot_df[available_cols].apply(pd.to_numeric, errors='coerce')
    normalized = values.copy()

    for col in normalized.columns:
        col_values = normalized[col]
        finite = col_values[np.isfinite(col_values)]
        if finite.empty:
            normalized[col] = 0.0
            continue
        lo = float(finite.min())
        hi = float(finite.max())
        if np.isclose(lo, hi):
            normalized[col] = 0.5
        else:
            normalized[col] = (col_values - lo) / (hi - lo)

    normalized = normalized.fillna(0.0)
    n_roads = len(plot_df)
    fig, ax = plt.subplots(figsize=(max(10, len(available_cols) * 1.2), min(max(6, n_roads * 0.12), 24)))
    im = ax.imshow(normalized.values, aspect='auto', cmap='viridis', vmin=0, vmax=1)
    ax.set_xticks(range(len(available_cols)))
    ax.set_xticklabels([c.replace('_', ' ') for c in available_cols], rotation=30, ha='right', fontsize=8)
    ax.set_xlabel('Analysis Metrics', fontsize=11)
    ax.set_ylabel('Road ID', fontsize=11)
    ax.set_title('Normalized Analysis Metrics Across Roads', fontsize=12)

    step = max(1, n_roads // 20)
    ticks = list(range(0, n_roads, step))
    ax.set_yticks(ticks)
    ax.set_yticklabels(plot_df['road_id'].iloc[ticks].tolist(), fontsize=7)

    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Normalized metric value', fontsize=9)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved analysis metrics heatmap → {output_path}")

def plot_robustness_traces(csv_path, output_dir, road_id):
    signals, _ = load_signals(csv_path)
    times = next(iter(signals.values())).times
    n = len(ALL_PROPERTIES)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3*n), sharex=True)
    for ax, (name, formula) in zip(axes, ALL_PROPERTIES.items()):
        try:
            trace = formula.robustness_trace(signals)
            widths = np.diff(times, prepend=times[0])
            color = ['#1a9850' if r >= 0 else '#d73027' for r in trace]
            ax.bar(times, trace, width=widths, color=color, alpha=0.7, align='edge')
            ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
            ax.set_ylabel('ρ', fontsize=8)
            label = formula.label if hasattr(formula, 'label') else name
            ax.set_title(label, fontsize=8)
            ax.grid(True, alpha=0.3)
        except Exception as e:
            ax.set_title(f"{name} — ERROR: {e}", fontsize=8)
    axes[-1].set_xlabel('Time (s)', fontsize=10)
    fig.suptitle(f'STL Robustness Traces — Road {road_id}', fontsize=12)
    plt.tight_layout()
    out = os.path.join(output_dir, f'trace_road_{road_id}.png')
    plt.savefig(out, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  Saved trace → {out}")

def main():
    parser = argparse.ArgumentParser(description='ADS STL Monitor')
    parser.add_argument('--data_dir',    default='/Users/ali/Documents/GitHub/udacity-test-generation/SensoDat/dynamic_data/a2')
    parser.add_argument('--output_dir',  default='/Users/ali/Documents/GitHub/udacity-test-generation/STL_Monitor_for_ADS_Behavior/results/d/a2')
    parser.add_argument('--max_roads',   type=int, default=None)
    parser.add_argument('--trace_roads', nargs='*', default=[])
    parser.add_argument('--recursive', action='store_true',
                        help='Recursively discover CSV files under --data_dir')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    script_dir = Path(__file__).resolve().parent
    fallback_data_dir = script_dir.parent / 'SensoDat' / 'dynamic_data'
    selected_data_dir = args.data_dir
    print(f"\n{'='*55}\n  ADS STL Monitor | data={selected_data_dir}\n{'='*55}\n")

    csv_files = find_csv_files(selected_data_dir, recursive=args.recursive)

    # If user runs with defaults and ./data is empty, auto-fallback to repo dataset.
    if not csv_files and args.data_dir == './data' and fallback_data_dir.exists():
        selected_data_dir = str(fallback_data_dir)
        print(f"  [INFO] No CSVs in ./data. Falling back to: {selected_data_dir}")
        csv_files = find_csv_files(selected_data_dir, recursive=True)

    if args.max_roads:
        csv_files = csv_files[:args.max_roads]

    road_lookup = {}
    for p in csv_files:
        road_lookup.setdefault(p.stem, []).append(p)

    if not csv_files:
        print("No CSV files found. Check --data_dir (or pass --recursive).")
        print("Example:")
        print("  python runner.py --data_dir /Users/ali/Documents/GitHub/udacity-test-generation/SensoDat/dynamic_data/a2/0.csv")
        return

    print("Evaluating STL properties...")
    rows = []
    for csv_path in csv_files:
        road_id = csv_path.stem
        try:
            signals, _ = load_signals(str(csv_path))
            res = evaluate_road(signals)
            row = {'road_id': road_id}
            for p, v in res.items():
                row[f'{p}_rho'] = v['robustness']
                row[f'{p}_ok'] = v['satisfied']
            rows.append(row)
        except Exception as e:
            print(f"  [WARN] Skipping {csv_path.name}: {e}")
    df = pd.DataFrame(rows)

    if df.empty:
        print("No roads evaluated. Check --data_dir."); return

    print("\nBuilding detailed violation timing reports...")
    detailed_df, summary_df = build_violation_reports(csv_files)
    metrics_df = build_metrics_table(csv_files)

    if not metrics_df.empty:
        df = df.merge(metrics_df, on='road_id', how='left')

    results_path = os.path.join(args.output_dir, 'results.csv')
    df.to_csv(results_path, index=False)
    print(f"\n  Results → {results_path}")

    detailed_path = os.path.join(args.output_dir, 'violations_detailed.csv')
    summary_path = os.path.join(args.output_dir, 'violations_summary.csv')
    metrics_path = os.path.join(args.output_dir, 'analysis_metrics.csv')
    detailed_df.to_csv(detailed_path, index=False)
    summary_df.to_csv(summary_path, index=False)
    metrics_df.to_csv(metrics_path, index=False)
    print(f"  Detailed violations → {detailed_path}")
    print(f"  Violation summary  → {summary_path}")
    print(f"  Analysis metrics    → {metrics_path}")

    clean_summary_paths = write_clean_summary_csvs(args.output_dir, df, summary_df, metrics_df)
    print(f"  Clean property summary → {clean_summary_paths['property_summary_path']}")
    print(f"  Clean metrics summary  → {clean_summary_paths['metrics_summary_path']}")
    print(f"  Clean overall summary  → {clean_summary_paths['overall_summary_path']}")
    print(f"  Top risk roads summary → {clean_summary_paths['top_risk_roads_path']}")

    print(f"\n{'='*55}\n  Summary ({len(df)} roads)\n{'='*55}")
    for p in ALL_PROPERTIES:
        sat  = df[f'{p}_ok'].mean()*100
        mrho = df[f'{p}_rho'].mean()
        print(f"  {p:<28}  satisfied: {sat:5.1f}%   mean ρ: {mrho:+.3f}")

    print("\nGenerating plots...")
    plot_violation_heatmap(df, os.path.join(args.output_dir, 'heatmap.png'))
    plot_summary_bar(df,       os.path.join(args.output_dir, 'violation_rates.png'))
    if not metrics_df.empty:
        plot_analysis_metrics_heatmap(metrics_df, os.path.join(args.output_dir, 'analysis_metrics_heatmap.png'))

    for road_id in args.trace_roads:
        matches = road_lookup.get(str(road_id), [])
        if matches:
            if len(matches) > 1:
                print(f"  [WARN] Multiple matches for road {road_id}; using {matches[0]}")
            plot_robustness_traces(str(matches[0]), args.output_dir, road_id)
        else:
            print(f"  [WARN] Road {road_id} not found.")

    print(f"\nDone. Outputs in: {args.output_dir}\n")

if __name__ == '__main__':
    main()
