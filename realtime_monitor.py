import csv
import html
import math
import os
from collections import deque
from datetime import datetime

import numpy as np

from ads_properties import ALL_PROPERTIES
from stl_monitor import Signal


class RealTimeSTLMonitor:
    def __init__(
        self,
        output_dir,
        view_mode="both",
        print_every=25,
        html_refresh_steps=25,
        alert_threshold=0.0,
    ):
        self.output_dir = output_dir
        self.view_mode = view_mode
        self.print_every = max(1, int(print_every))
        self.html_refresh_steps = max(1, int(html_refresh_steps))
        self.alert_threshold = float(alert_threshold)

        os.makedirs(self.output_dir, exist_ok=True)

        self.property_names = list(ALL_PROPERTIES.keys())
        self.stream_csv_path = os.path.join(self.output_dir, "realtime_stream.csv")
        self.alerts_csv_path = os.path.join(self.output_dir, "realtime_alerts.csv")
        self.episode_summary_path = os.path.join(self.output_dir, "realtime_episode_summary.csv")
        self.dashboard_html_path = os.path.join(self.output_dir, "realtime_dashboard.html")

        self._stream_headers = [
            "wall_time",
            "episode_idx",
            "road_id",
            "sim_time_s",
            "speed",
            "cte",
            "str_angle",
            "hdg_err",
            "curvature",
        ]
        for name in self.property_names:
            self._stream_headers.extend([f"{name}_rho", f"{name}_status"])

        self._alert_headers = [
            "wall_time",
            "episode_idx",
            "road_id",
            "sim_time_s",
            "property",
            "rho",
            "status",
        ]

        self._episode_headers = [
            "wall_time",
            "episode_idx",
            "road_id",
            "agent_type",
            "model_name",
            "prioritization_method",
            "episode_length",
            "episode_time_s",
            "success",
            "max_speed",
            "max_abs_cte",
            "violated_properties",
            "pending_properties",
        ]
        for name in self.property_names:
            self._episode_headers.extend([f"{name}_rho", f"{name}_status"])

        self._ensure_csv_headers(self.stream_csv_path, self._stream_headers)
        self._ensure_csv_headers(self.alerts_csv_path, self._alert_headers)
        self._ensure_csv_headers(self.episode_summary_path, self._episode_headers)

        self._recent_rows = deque(maxlen=200)
        self._active_property_status = {}
        self._last_eval = None
        self._last_episode_summary = None
        self._episode_meta = None
        self._steps_in_episode = 0

        self._reset_buffers()
        self._write_dashboard_html()

    def _ensure_csv_headers(self, csv_path, headers):
        if os.path.exists(csv_path) and os.path.getsize(csv_path) > 0:
            return
        with open(csv_path, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def _reset_buffers(self):
        self._times = []
        self._speed = []
        self._cte = []
        self._str_angle = []
        self._hdg_err = []
        self._curvature = []

    def start_episode(self, episode_idx, road_id, agent_type, model_name, prioritization_method):
        self._reset_buffers()
        self._steps_in_episode = 0
        self._active_property_status = {name: "pending" for name in self.property_names}
        self._last_eval = None
        self._episode_meta = {
            "episode_idx": int(episode_idx),
            "road_id": str(road_id),
            "agent_type": str(agent_type),
            "model_name": str(model_name),
            "prioritization_method": str(prioritization_method),
        }

    def _build_signals(self):
        times = np.asarray(self._times, dtype=float)
        return {
            "speed": Signal(times, np.asarray(self._speed, dtype=float)),
            "cte": Signal(times, np.asarray(self._cte, dtype=float)),
            "str_angle": Signal(times, np.asarray(self._str_angle, dtype=float)),
            "hdg_err": Signal(times, np.asarray(self._hdg_err, dtype=float)),
            "curvature": Signal(times, np.asarray(self._curvature, dtype=float)),
        }

    def _evaluate_properties(self):
        if len(self._times) < 2:
            result = {}
            for name in self.property_names:
                result[name] = {"rho": float("nan"), "status": "pending"}
            return result

        signals = self._build_signals()
        result = {}
        for name, formula in ALL_PROPERTIES.items():
            try:
                rho = float(formula.robustness(signals, t=0.0))
                if math.isinf(rho) or math.isnan(rho):
                    status = "pending"
                    out_rho = float("nan")
                else:
                    status = "violated" if rho < self.alert_threshold else "satisfied"
                    out_rho = round(rho, 5)
                result[name] = {"rho": out_rho, "status": status}
            except Exception:
                result[name] = {"rho": float("nan"), "status": "pending"}
        return result

    def update(self, timestamp, speed, cte, str_angle, hdg_err, curvature):
        if self._episode_meta is None:
            raise RuntimeError("Call start_episode() before update().")

        self._times.append(float(timestamp))
        self._speed.append(float(speed))
        self._cte.append(float(cte))
        self._str_angle.append(float(str_angle))
        self._hdg_err.append(float(hdg_err))
        self._curvature.append(float(curvature))
        self._steps_in_episode += 1

        eval_result = self._evaluate_properties()
        self._last_eval = eval_result

        newly_violated = []
        for name in self.property_names:
            old = self._active_property_status.get(name, "pending")
            new = eval_result[name]["status"]
            if old != "violated" and new == "violated":
                newly_violated.append((name, eval_result[name]["rho"]))
            self._active_property_status[name] = new

        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        stream_row = {
            "wall_time": now_str,
            "episode_idx": self._episode_meta["episode_idx"],
            "road_id": self._episode_meta["road_id"],
            "sim_time_s": round(float(timestamp), 5),
            "speed": round(float(speed), 5),
            "cte": round(float(cte), 5),
            "str_angle": round(float(str_angle), 5),
            "hdg_err": round(float(hdg_err), 5),
            "curvature": round(float(curvature), 7),
        }
        for name in self.property_names:
            stream_row[f"{name}_rho"] = eval_result[name]["rho"]
            stream_row[f"{name}_status"] = eval_result[name]["status"]

        self._append_dict_row(self.stream_csv_path, self._stream_headers, stream_row)
        self._recent_rows.append(stream_row)

        for name, rho in newly_violated:
            alert_row = {
                "wall_time": now_str,
                "episode_idx": self._episode_meta["episode_idx"],
                "road_id": self._episode_meta["road_id"],
                "sim_time_s": round(float(timestamp), 5),
                "property": name,
                "rho": rho,
                "status": "violated",
            }
            self._append_dict_row(self.alerts_csv_path, self._alert_headers, alert_row)

        should_print = self._steps_in_episode % self.print_every == 0 or len(newly_violated) > 0
        should_refresh_html = self._steps_in_episode % self.html_refresh_steps == 0 or len(newly_violated) > 0

        if should_print and self.view_mode in ("terminal", "both"):
            violated = [p for p in self.property_names if eval_result[p]["status"] == "violated"]
            pending = [p for p in self.property_names if eval_result[p]["status"] == "pending"]
            msg = (
                f"[RT-STL] ep={self._episode_meta['episode_idx']} road={self._episode_meta['road_id']} "
                f"t={timestamp:.2f}s step={self._steps_in_episode} "
                f"violated={len(violated)} pending={len(pending)}"
            )
            print(msg)
            for name in newly_violated:
                prop_name, rho = name
                print(f"  [RT-STL-ALERT] {prop_name} violated (rho={rho})")

        if should_refresh_html and self.view_mode in ("html", "both"):
            self._write_dashboard_html()

        return {
            "evaluation": eval_result,
            "newly_violated": newly_violated,
        }

    def end_episode(self, success, episode_time_s, episode_length, max_speed, max_abs_cte):
        if self._episode_meta is None:
            return None

        final_eval = self._last_eval or self._evaluate_properties()
        violated_count = sum(1 for p in self.property_names if final_eval[p]["status"] == "violated")
        pending_count = sum(1 for p in self.property_names if final_eval[p]["status"] == "pending")

        summary = {
            "wall_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "episode_idx": self._episode_meta["episode_idx"],
            "road_id": self._episode_meta["road_id"],
            "agent_type": self._episode_meta["agent_type"],
            "model_name": self._episode_meta["model_name"],
            "prioritization_method": self._episode_meta["prioritization_method"],
            "episode_length": int(episode_length),
            "episode_time_s": round(float(episode_time_s), 5),
            "success": int(bool(success)),
            "max_speed": round(float(max_speed), 5),
            "max_abs_cte": round(float(max_abs_cte), 5),
            "violated_properties": violated_count,
            "pending_properties": pending_count,
        }

        for name in self.property_names:
            summary[f"{name}_rho"] = final_eval[name]["rho"]
            summary[f"{name}_status"] = final_eval[name]["status"]

        self._append_dict_row(self.episode_summary_path, self._episode_headers, summary)
        self._last_episode_summary = summary

        if self.view_mode in ("terminal", "both"):
            print(
                f"[RT-STL-END] ep={summary['episode_idx']} road={summary['road_id']} "
                f"violated={violated_count} pending={pending_count} success={summary['success']}"
            )

        if self.view_mode in ("html", "both"):
            self._write_dashboard_html()

        self._episode_meta = None
        return summary

    def close(self):
        if self.view_mode in ("html", "both"):
            self._write_dashboard_html()

    def _append_dict_row(self, path, headers, row_dict):
        with open(path, mode="a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writerow(row_dict)

    def _write_dashboard_html(self):
        active = self._episode_meta or {}
        title = "Real-Time STL Dashboard"

        status_rows = ""
        if self._last_eval:
            for name in self.property_names:
                rho = self._last_eval[name]["rho"]
                status = self._last_eval[name]["status"]
                color = "#1a7f37" if status == "satisfied" else "#b42318" if status == "violated" else "#9a6700"
                rho_text = "NaN" if (isinstance(rho, float) and math.isnan(rho)) else str(rho)
                status_rows += (
                    f"<tr><td>{html.escape(name)}</td>"
                    f"<td>{html.escape(rho_text)}</td>"
                    f"<td style='color:{color};font-weight:700'>{html.escape(status)}</td></tr>"
                )

        recent_rows_html = ""
        for row in list(self._recent_rows)[-30:]:
            recent_rows_html += (
                "<tr>"
                f"<td>{html.escape(str(row.get('wall_time', '')))}</td>"
                f"<td>{html.escape(str(row.get('episode_idx', '')))}</td>"
                f"<td>{html.escape(str(row.get('road_id', '')))}</td>"
                f"<td>{html.escape(str(row.get('sim_time_s', '')))}</td>"
                f"<td>{html.escape(str(row.get('speed', '')))}</td>"
                f"<td>{html.escape(str(row.get('cte', '')))}</td>"
                "</tr>"
            )

        summary_html = ""
        if self._last_episode_summary:
            summary_html = (
                "<div class='card'><h3>Last Episode Summary</h3>"
                f"<p>Episode: {html.escape(str(self._last_episode_summary.get('episode_idx')))} | "
                f"Road: {html.escape(str(self._last_episode_summary.get('road_id')))}</p>"
                f"<p>Violated Properties: {html.escape(str(self._last_episode_summary.get('violated_properties')))} | "
                f"Pending: {html.escape(str(self._last_episode_summary.get('pending_properties')))} | "
                f"Success: {html.escape(str(self._last_episode_summary.get('success')))}</p>"
                "</div>"
            )

        html_text = f"""<!doctype html>
<html>
<head>
  <meta charset='utf-8'>
  <meta http-equiv='refresh' content='2'>
  <title>{title}</title>
  <style>
    body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; margin: 24px; background: #f7f8fa; color: #1f2328; }}
    h1 {{ margin-bottom: 6px; }}
    .sub {{ color: #57606a; margin-top: 0; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
    .card {{ background: #fff; border: 1px solid #d0d7de; border-radius: 10px; padding: 14px; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
    th, td {{ border-bottom: 1px solid #d8dee4; padding: 6px 8px; text-align: left; }}
    th {{ background: #f6f8fa; }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  <p class='sub'>Auto-refresh every 2s | Output dir: {html.escape(self.output_dir)}</p>

  <div class='card'>
    <h3>Current Episode</h3>
    <p class='mono'>episode_idx={html.escape(str(active.get('episode_idx', '-')))} road_id={html.escape(str(active.get('road_id', '-')))} steps={self._steps_in_episode}</p>
  </div>

  {summary_html}

  <div class='grid'>
    <div class='card'>
      <h3>Current STL Property Status</h3>
      <table>
        <thead><tr><th>Property</th><th>Robustness (rho)</th><th>Status</th></tr></thead>
        <tbody>{status_rows}</tbody>
      </table>
    </div>

    <div class='card'>
      <h3>Recent Telemetry (Last 30 Rows)</h3>
      <table>
        <thead><tr><th>Wall Time</th><th>Ep</th><th>Road</th><th>Sim t</th><th>Speed</th><th>CTE</th></tr></thead>
        <tbody>{recent_rows_html}</tbody>
      </table>
    </div>
  </div>
</body>
</html>
"""

        with open(self.dashboard_html_path, mode="w", encoding="utf-8") as f:
            f.write(html_text)
