#!/usr/bin/env python3
"""
Generate an interactive HTML viewer for PF-IMU MATLAB vs local comparisons.

Output: pf_imu_compare_results_50_150s/pf_imu_interactive_viewer.html
"""

from pathlib import Path
import json
import pandas as pd


BASE = Path(__file__).resolve().parent
RESULT_DIR = BASE / "pf_imu_compare_results_50_150s"
TRACES_CSV = RESULT_DIR / "pf_imu_zhemin_vs_local_traces_50_150s.csv"
WINDOW_CSV = RESULT_DIR / "pf_imu_windowed_5s_metrics.csv"
OUT_HTML = RESULT_DIR / "pf_imu_interactive_viewer.html"


def _load_data():
    traces = pd.read_csv(TRACES_CSV)
    windows = pd.read_csv(WINDOW_CSV)
    return traces, windows


def _side_payload(traces: pd.DataFrame, windows: pd.DataFrame, side: str):
    s = side.upper()
    payload = {
        "t": traces["t_s"].round(6).tolist(),
        "mat": traces[f"mat_tau_{s}"].tolist(),
        "local_ref_mc": traces[f"local_ref_mcmean_{s}"].tolist(),
        "local_dep_mc": traces[f"local_deploy_mcmean_{s}"].tolist(),
    }
    payload["err_ref"] = (traces[f"local_ref_mcmean_{s}"] - traces[f"mat_tau_{s}"]).tolist()
    payload["err_dep"] = (traces[f"local_deploy_mcmean_{s}"] - traces[f"mat_tau_{s}"]).tolist()

    sub = windows[windows["side"] == s].sort_values("window_start_s")
    ref = sub[sub["pair"] == "matlab_vs_ref_mc"]
    dep = sub[sub["pair"] == "matlab_vs_deploy_mc"]

    merged = dep.merge(
        ref[["window_start_s", "corr_0lag", "rmse_0lag"]],
        on="window_start_s",
        suffixes=("_dep", "_ref"),
    ).sort_values("window_start_s")

    payload["win_center_s"] = ((merged["window_start_s"] + 2.5).round(3)).tolist()
    payload["corr_ref"] = merged["corr_0lag_ref"].tolist()
    payload["corr_dep"] = merged["corr_0lag_dep"].tolist()
    payload["rmse_ref"] = merged["rmse_0lag_ref"].tolist()
    payload["rmse_dep"] = merged["rmse_0lag_dep"].tolist()
    payload["lag_dep_ms"] = merged["best_lag_ms"].tolist()
    return payload


def _build_html(data_l: dict, data_r: dict):
    data_json = json.dumps({"L": data_l, "R": data_r}, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>PF-IMU MATLAB vs Local Interactive Viewer</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    body {{
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      margin: 12px;
      background: #111418;
      color: #e6edf3;
    }}
    .toolbar {{
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 10px;
      flex-wrap: wrap;
    }}
    select, button {{
      background: #1f2937;
      color: #e6edf3;
      border: 1px solid #374151;
      border-radius: 8px;
      padding: 6px 10px;
      cursor: pointer;
    }}
    .mode-btn.active {{
      background: #0ea5e9;
      border-color: #38bdf8;
      color: #001018;
      font-weight: 700;
    }}
    .hint {{
      color: #93a4b8;
      font-size: 13px;
    }}
    .plotbox {{
      width: 100%;
      height: 86vh;
      min-height: 640px;
      border: 1px solid #334155;
      border-radius: 10px;
      overflow: hidden;
      background: #0b1118;
    }}
    .hidden {{
      display: none;
    }}
  </style>
</head>
<body>
  <div class="toolbar">
    <button id="btnTorqueOnly" class="mode-btn active">Torque Only</button>
    <button id="btnAllPanels" class="mode-btn">All Panels</button>
    <label>腿侧:
      <select id="sideSel">
        <option value="L">Left</option>
        <option value="R">Right</option>
      </select>
    </label>
    <button id="btnReset">重置视图</button>
    <span class="hint">支持：滚轮缩放、框选放大、平移、双击复位</span>
  </div>
  <div id="plotTorque" class="plotbox"></div>
  <div id="plotAll" class="plotbox hidden"></div>

  <script>
    const DATA = {data_json};
    const PLOT_TORQUE = "plotTorque";
    const PLOT_ALL = "plotAll";
    let mode = "torque";

    function torqueTraces(d) {{
      return [
        {{
          x: d.t, y: d.mat, mode: "lines", name: "MATLAB torque", line: {{width: 2.2, color: "#10b981"}}
        }},
        {{
          x: d.t, y: d.local_ref_mc, mode: "lines", name: "Local Ref (MC mean)", line: {{width: 1.9, color: "#38bdf8"}}
        }},
        {{
          x: d.t, y: d.local_dep_mc, mode: "lines", name: "Local Deploy (MC mean)", line: {{width: 1.9, color: "#f59e0b"}}
        }}
      ];
    }}

    function allTraces(d) {{
      const torqueMat = {{
        x: d.t, y: d.mat, mode: "lines", name: "MATLAB torque", line: {{width: 2, color: "#10b981"}}, xaxis: "x", yaxis: "y"
      }};
      const torqueRef = {{
        x: d.t, y: d.local_ref_mc, mode: "lines", name: "Local Ref (MC mean)", line: {{width: 1.8, color: "#38bdf8"}}, xaxis: "x", yaxis: "y"
      }};
      const torqueDep = {{
        x: d.t, y: d.local_dep_mc, mode: "lines", name: "Local Deploy (MC mean)", line: {{width: 1.8, color: "#f59e0b"}}, xaxis: "x", yaxis: "y"
      }};
      const errRef = {{
        x: d.t, y: d.err_ref, mode: "lines", name: "Error: Ref - MATLAB", line: {{width: 1.4, color: "#22d3ee"}}, xaxis: "x2", yaxis: "y2"
      }};
      const errDep = {{
        x: d.t, y: d.err_dep, mode: "lines", name: "Error: Deploy - MATLAB", line: {{width: 1.4, color: "#fb7185"}}, xaxis: "x2", yaxis: "y2"
      }};
      const corrRef = {{
        x: d.win_center_s, y: d.corr_ref, type: "scatter", mode: "lines+markers", name: "5s Corr Ref", line: {{width: 1.6, color: "#60a5fa"}}, xaxis: "x3", yaxis: "y3"
      }};
      const corrDep = {{
        x: d.win_center_s, y: d.corr_dep, type: "scatter", mode: "lines+markers", name: "5s Corr Deploy", line: {{width: 1.6, color: "#f97316"}}, xaxis: "x3", yaxis: "y3"
      }};
      const rmseRef = {{
        x: d.win_center_s, y: d.rmse_ref, type: "scatter", mode: "lines+markers", name: "5s RMSE Ref (Nm)", line: {{width: 1.6, color: "#34d399", dash: "dot"}}, xaxis: "x4", yaxis: "y4"
      }};
      const rmseDep = {{
        x: d.win_center_s, y: d.rmse_dep, type: "scatter", mode: "lines+markers", name: "5s RMSE Deploy (Nm)", line: {{width: 1.6, color: "#ef4444", dash: "dot"}}, xaxis: "x4", yaxis: "y4"
      }};
      return [torqueMat, torqueRef, torqueDep, errRef, errDep, corrRef, corrDep, rmseRef, rmseDep];
    }}

    function torqueLayout(side) {{
      return {{
        paper_bgcolor: "#0b1118",
        plot_bgcolor: "#0b1118",
        font: {{color: "#e5e7eb"}},
        margin: {{l: 70, r: 30, t: 130, b: 80}},
        title: {{text: `PF-IMU Torque Only (Side: ${{side}})`, x: 0.01, xanchor: "left", y: 0.96, yanchor: "top"}},
        legend: {{orientation: "h", x: 0.01, xanchor: "left", y: 1.18, yanchor: "top"}},
        hovermode: "x unified",
        xaxis: {{
          title: "Time (s)",
          showgrid: true,
          gridcolor: "#223244",
          rangeslider: {{visible: true, thickness: 0.11, bgcolor: "#0f172a", bordercolor: "#334155", borderwidth: 1}}
        }},
        yaxis: {{title: "Torque (Nm)", zeroline: true, zerolinecolor: "#374151", showgrid: true, gridcolor: "#223244"}},
      }};
    }}

    function allLayout(side) {{
      return {{
        paper_bgcolor: "#0b1118",
        plot_bgcolor: "#0b1118",
        font: {{color: "#e5e7eb"}},
        margin: {{l: 70, r: 30, t: 130, b: 80}},
        title: {{text: `PF-IMU vs MATLAB (Side: ${{side}})`, x: 0.01, xanchor: "left", y: 0.96, yanchor: "top"}},
        legend: {{orientation: "h", x: 0.01, xanchor: "left", y: 1.18, yanchor: "top"}},
        grid: {{rows: 4, columns: 1, pattern: "independent", roworder: "top to bottom"}},
        hovermode: "x unified",

        xaxis:  {{
          title: "Time (s)",
          showgrid: true,
          gridcolor: "#223244",
          rangeslider: {{visible: true, thickness: 0.08, bgcolor: "#0f172a", bordercolor: "#334155", borderwidth: 1}}
        }},
        yaxis:  {{title: "Torque (Nm)", zeroline: true, zerolinecolor: "#374151", showgrid: true, gridcolor: "#223244"}},
        xaxis2: {{title: "Time (s)", showgrid: true, gridcolor: "#223244"}},
        yaxis2: {{title: "Error (Nm)", zeroline: true, zerolinecolor: "#4b5563", showgrid: true, gridcolor: "#223244"}},
        xaxis3: {{title: "Window Center (s)", showgrid: true, gridcolor: "#223244"}},
        yaxis3: {{title: "5s Corr", range: [0.3, 1.0], showgrid: true, gridcolor: "#223244"}},
        xaxis4: {{title: "Window Center (s)", showgrid: true, gridcolor: "#223244"}},
        yaxis4: {{title: "5s RMSE (Nm)", showgrid: true, gridcolor: "#223244"}},
      }};
    }}

    function render(side) {{
      const d = DATA[side];
      Plotly.react(PLOT_TORQUE, torqueTraces(d), torqueLayout(side), {{
        responsive: true,
        displaylogo: false,
        scrollZoom: true
      }});
      Plotly.react(PLOT_ALL, allTraces(d), allLayout(side), {{
        responsive: true,
        displaylogo: false,
        scrollZoom: true
      }});
    }}

    const sideSel = document.getElementById("sideSel");
    const btnReset = document.getElementById("btnReset");
    const btnTorqueOnly = document.getElementById("btnTorqueOnly");
    const btnAllPanels = document.getElementById("btnAllPanels");
    const plotTorque = document.getElementById(PLOT_TORQUE);
    const plotAll = document.getElementById(PLOT_ALL);

    function setMode(nextMode) {{
      mode = nextMode;
      if (mode === "torque") {{
        plotTorque.classList.remove("hidden");
        plotAll.classList.add("hidden");
        btnTorqueOnly.classList.add("active");
        btnAllPanels.classList.remove("active");
      }} else {{
        plotAll.classList.remove("hidden");
        plotTorque.classList.add("hidden");
        btnAllPanels.classList.add("active");
        btnTorqueOnly.classList.remove("active");
      }}
      window.dispatchEvent(new Event("resize"));
    }}

    sideSel.addEventListener("change", () => render(sideSel.value));
    btnTorqueOnly.addEventListener("click", () => setMode("torque"));
    btnAllPanels.addEventListener("click", () => setMode("all"));
    btnReset.addEventListener("click", () => {{
      if (mode === "torque") {{
        Plotly.relayout(PLOT_TORQUE, {{"xaxis.autorange": true, "yaxis.autorange": true}});
      }} else {{
        Plotly.relayout(PLOT_ALL, {{
          "xaxis.autorange": true, "xaxis2.autorange": true, "xaxis3.autorange": true, "xaxis4.autorange": true,
          "yaxis.autorange": true, "yaxis2.autorange": true, "yaxis3.autorange": true, "yaxis4.autorange": true
        }});
      }}
    }});

    render("L");
    setMode("torque");
  </script>
</body>
</html>
"""


def main():
    traces, windows = _load_data()
    html = _build_html(_side_payload(traces, windows, "L"), _side_payload(traces, windows, "R"))
    OUT_HTML.write_text(html, encoding="utf-8")
    print(f"[saved] {OUT_HTML}")


if __name__ == "__main__":
    main()
