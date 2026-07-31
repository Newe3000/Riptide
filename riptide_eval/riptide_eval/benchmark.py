#!/usr/bin/env python3
"""Benchmark the Riptide control zoo.

For every (control_law x disturbance) combination this:
  1. launches the full MuJoCo + ros2_control stack,
  2. lets it settle, then records /riptide/control_debug (EE error, torque,
     solve time) and /riptide/odom (base pose) for a fixed window,
  3. tears the stack down,
  4. computes station-keeping / effort / timing metrics.

The disturbance generator uses a fixed `seed`, so every controller faces the
*same* turbulent-current realization -- a fair, reproducible comparison.

Outputs (in --output-dir):
  summary.csv                      one row per (law, scenario) with all metrics
  raw_<law>_<scenario>.csv         per-run time series (t, ee/base error, torque)
  ee_error_<scenario>.png          EE position-error norm vs time, laws overlaid
  summary_ee_rms.png               grouped bars: EE position RMS error
  summary_effort.png               grouped bars: RMS joint torque (effort)
  summary_solvetime.png            grouped bars: mean compute() time

Usage:
  ros2 run riptide_eval benchmark
  ros2 run riptide_eval benchmark --laws impedance,lqr,mpc \
      --scenarios sinusoid,stochastic --duration 20 --output-dir ~/riptide_bench
"""
import argparse
import csv
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from riptide_msgs.msg import ControlDebug

PROCS = [
    "ros2_control_node", "robot_state_publisher", "mock_base_tf",
    "disturbance_generator", "static_transform_pub", "rviz2", "bin/ros2 launch",
]


def kill_stack():
    """Reap any running sim processes (belt-and-braces; the launch also reaps)."""
    for name in PROCS:
        subprocess.run(["pkill", "-9", "-f", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


class Recorder(Node):
    def __init__(self):
        super().__init__("riptide_benchmark_recorder")
        self.t, self.err, self.tau, self.solve, self.law = [], [], [], [], None
        self.bt, self.bp = [], []
        self.create_subscription(ControlDebug, "/riptide/control_debug", self._cd, 50)
        self.create_subscription(Odometry, "/riptide/odom", self._od, 50)
        self.t0 = time.monotonic()

    def _cd(self, m):
        self.t.append(time.monotonic() - self.t0)
        self.err.append(list(m.ee_pose_error))
        self.tau.append(list(m.tau))
        self.solve.append(m.solve_time_ms)
        self.law = m.control_law

    def _od(self, m):
        self.bt.append(time.monotonic() - self.t0)
        p = m.pose.pose.position
        self.bp.append([p.x, p.y, p.z])


def run_combo(law, scenario, args, outdir):
    kill_stack()
    time.sleep(2.0)
    logf = open(outdir / f"log_{law}_{scenario}.txt", "w")
    cmd = [
        "ros2", "launch", "riptide_bringup", "sim.launch.py",
        "use_mock_hardware:=false", "controller:=ee", f"control_law:={law}",
        "base_control:=true", "rviz:=false", f"disturbance:={scenario}",
        f"disturbance_amplitude:={args.amplitude}",
    ]
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                            start_new_session=True)
    print(f"  launching {law} / {scenario} (settle {args.settle}s, "
          f"record {args.duration}s) ...", flush=True)
    time.sleep(args.settle)

    rec = Recorder()
    end = time.monotonic() + args.duration
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(rec, timeout_sec=0.1)

    # Teardown: SIGINT the launch process group, then hard-reap.
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        pass
    result = _metrics(law, scenario, rec)
    rec.destroy_node()
    logf.close()
    kill_stack()
    time.sleep(2.0)
    return result


def _metrics(law, scenario, rec):
    n = len(rec.err)
    if n < 10:
        print(f"  WARNING: only {n} ControlDebug samples for {law}/{scenario} "
              f"(controller may have failed to start).", flush=True)
        return None

    t = np.asarray(rec.t)
    err = np.asarray(rec.err)                       # n x 6
    pos_norm = np.linalg.norm(err[:, 0:3], axis=1)  # EE position error [m]
    ori_norm = np.linalg.norm(err[:, 3:6], axis=1)  # EE orientation error [rad]
    tau = np.asarray(rec.tau)
    tau_norm = np.linalg.norm(tau, axis=1)
    solve = np.asarray(rec.solve)

    if len(rec.bp) >= 2:
        bp = np.asarray(rec.bp)
        base_dev = np.linalg.norm(bp - bp.mean(axis=0), axis=1)
        base_rms = float(np.sqrt(np.mean(base_dev ** 2)))
    else:
        base_rms = float("nan")

    return {
        "control_law": rec.law or law,
        "scenario": scenario,
        "samples": n,
        "ee_pos_rms_m": float(np.sqrt(np.mean(pos_norm ** 2))),
        "ee_pos_max_m": float(np.max(pos_norm)),
        "ee_ori_rms_rad": float(np.sqrt(np.mean(ori_norm ** 2))),
        "tau_rms_Nm": float(np.sqrt(np.mean(tau_norm ** 2))),
        "base_pos_rms_m": base_rms,
        "solve_ms_mean": float(np.mean(solve)),
        "solve_ms_max": float(np.max(solve)),
        "_t": t, "_pos_norm": pos_norm, "_tau_norm": tau_norm,   # for plots/raw
    }


def write_outputs(results, laws, scenarios, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = ["control_law", "scenario", "samples", "ee_pos_rms_m", "ee_pos_max_m",
            "ee_ori_rms_rad", "tau_rms_Nm", "base_pos_rms_m",
            "solve_ms_mean", "solve_ms_max"]
    with open(outdir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in results.values():
            if r is None:
                continue
            w.writerow({k: r[k] for k in cols})
    print(f"wrote {outdir/'summary.csv'}", flush=True)

    # Per-run raw time series.
    for (law, scen), r in results.items():
        if r is None:
            continue
        with open(outdir / f"raw_{law}_{scen}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "ee_pos_err_m", "tau_norm_Nm"])
            for i in range(len(r["_t"])):
                w.writerow([r["_t"][i], r["_pos_norm"][i], r["_tau_norm"][i]])

    # EE-error time series overlay, one figure per scenario.
    for scen in scenarios:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        any_line = False
        for law in laws:
            r = results.get((law, scen))
            if r is None:
                continue
            ax.plot(r["_t"], r["_pos_norm"] * 100.0, label=law, linewidth=1.3)
            any_line = True
        if not any_line:
            plt.close(fig)
            continue
        ax.set_xlabel("time [s]")
        ax.set_ylabel("EE position error [cm]")
        ax.set_title(f"EE station-keeping error — disturbance: {scen}")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(outdir / f"ee_error_{scen}.png", dpi=130)
        plt.close(fig)

    # Grouped bar charts.
    def grouped_bar(metric, ylabel, title, fname, scale=1.0):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        x = np.arange(len(laws))
        width = 0.8 / max(len(scenarios), 1)
        for j, scen in enumerate(scenarios):
            vals = [(results.get((law, scen)) or {}).get(metric, np.nan) for law in laws]
            vals = [v * scale if v is not None else np.nan for v in vals]
            ax.bar(x + j * width, vals, width, label=scen)
        ax.set_xticks(x + width * (len(scenarios) - 1) / 2)
        ax.set_xticklabels(laws)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(outdir / fname, dpi=130)
        plt.close(fig)

    grouped_bar("ee_pos_rms_m", "EE position RMS error [cm]",
                "Station-keeping accuracy (lower is better)",
                "summary_ee_rms.png", scale=100.0)
    grouped_bar("tau_rms_Nm", "RMS joint torque [Nm]",
                "Control effort (lower is better)", "summary_effort.png")
    grouped_bar("solve_ms_mean", "mean compute() time [ms]",
                "Per-cycle solver cost", "summary_solvetime.png")
    print(f"wrote plots to {outdir}", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Riptide control-zoo benchmark")
    ap.add_argument("--laws", default="impedance,lqr,mpc")
    ap.add_argument("--scenarios", default="sinusoid,stochastic")
    ap.add_argument("--settle", type=float, default=6.0, help="settle time [s]")
    ap.add_argument("--duration", type=float, default=15.0, help="record window [s]")
    ap.add_argument("--amplitude", type=float, default=80.0, help="disturbance force [N]")
    ap.add_argument("--output-dir", default="./riptide_benchmark")
    args = ap.parse_args(argv)

    laws = [s.strip() for s in args.laws.split(",") if s.strip()]
    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    outdir = Path(os.path.expanduser(args.output_dir)).resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    print(f"Benchmark: laws={laws} scenarios={scenarios} -> {outdir}", flush=True)

    rclpy.init()
    results = {}
    try:
        for scen in scenarios:
            for law in laws:
                results[(law, scen)] = run_combo(law, scen, args, outdir)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        kill_stack()

    write_outputs(results, laws, scenarios, outdir)

    print("\n=== summary ===", flush=True)
    for (law, scen), r in results.items():
        if r is None:
            print(f"  {law:10s} {scen:12s}  FAILED")
        else:
            print(f"  {law:10s} {scen:12s}  EE_rms={r['ee_pos_rms_m']*100:5.1f} cm  "
                  f"EE_max={r['ee_pos_max_m']*100:5.1f} cm  "
                  f"tau_rms={r['tau_rms_Nm']:5.1f} Nm  "
                  f"solve={r['solve_ms_mean']:.2f} ms", flush=True)


if __name__ == "__main__":
    main()
