#!/usr/bin/env python3
"""Optional result plots. Core algorithms remain in C++."""

from pathlib import Path
import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_trace(csv_path: Path, out_dir: Path) -> None:
    df = pd.read_csv(csv_path)
    controller = csv_path.stem.replace("trace_", "")
    q_cols = [c for c in df.columns if c.startswith("q") and not c.startswith("q_des")]
    qd_cols = [c for c in df.columns if c.startswith("q_des")]
    if not q_cols or len(q_cols) != len(qd_cols):
        raise ValueError(f"Unexpected trace columns in {csv_path}")

    fig, ax = plt.subplots(figsize=(10, 5))
    for q, qd in zip(q_cols, qd_cols):
        ax.plot(df["t"], df[qd], linewidth=1.0, alpha=0.6)
        ax.plot(df["t"], df[q], linewidth=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("joint position [rad]")
    ax.set_title(f"Tracking: {controller}")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_dir / f"tracking_{controller}.png", dpi=160)
    plt.close(fig)


def plot_control_rmse(results_dir: Path) -> None:
    path = results_dir / "control_metrics.csv"
    if not path.exists():
        return
    df = pd.read_csv(path)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.bar(df["controller"], df["overall_rmse_deg"])
    ax.axhline(1.0, linestyle="--", linewidth=1.0, label="A2 limit: 1 deg")
    ax.set_ylabel("overall tracking RMSE [deg]")
    ax.set_title("Closed-loop tracking comparison")
    ax.grid(True, axis="y", alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir / "control_rmse.png", dpi=160)
    plt.close(fig)


def plot_singular_values(results_dir: Path) -> None:
    path = results_dir / "singular_values.csv"
    if not path.exists():
        return
    df = pd.read_csv(path)
    x = np.arange(1, len(df) + 1)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.semilogy(x, df["singular_value"], marker="o", markersize=3, linewidth=1.0)
    retained = df["retained"].astype(bool).to_numpy()
    if retained.any():
        cutoff = df.loc[retained, "singular_value"].iloc[-1]
        ax.axhline(cutoff, linestyle="--", linewidth=1.0, label="smallest retained")
    ax.set_xlabel("singular-value index")
    ax.set_ylabel("scaled singular value")
    ax.set_title("Identification regressor spectrum")
    ax.grid(True, which="both", alpha=0.2)
    if retained.any():
        ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir / "singular_values.png", dpi=160)
    plt.close(fig)


def plot_validation_nrmse(results_dir: Path) -> None:
    path = results_dir / "validation_metrics.csv"
    if not path.exists():
        return
    df = pd.read_csv(path)
    joint_number = pd.to_numeric(df["joint"], errors="coerce")
    joint_df = df.loc[joint_number.notna()].copy()
    joints = joint_number.loc[joint_number.notna()].astype(int).to_numpy()
    width = 0.36
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(joints - width / 2, 100.0 * joint_df["identified_nrmse"], width=width, label="identified")
    ax.bar(joints + width / 2, 100.0 * joint_df["nominal_nrmse"], width=width, label="nominal")
    ax.set_xticks(joints)
    ax.set_xlabel("joint")
    ax.set_ylabel("per-joint torque NRMSE [%] (diagnostic)")

    title = "Unseen-trajectory torque prediction — per-joint NRMSE"
    overall = df.loc[df["joint"].astype(str) == "overall", "identified_nrmse"]
    if not overall.empty:
        title += f"\nA1 uses overall NRMSE = {100.0 * float(overall.iloc[0]):.4f}%"
    else:
        acceptance_path = results_dir / "acceptance.csv"
        if acceptance_path.exists():
            acceptance = pd.read_csv(acceptance_path)
            a1 = acceptance.loc[acceptance["criterion"] == "A1", "metric"]
            if not a1.empty:
                title += f"\nA1 uses overall NRMSE = {100.0 * float(a1.iloc[0]):.4f}%"
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir / "validation_nrmse.png", dpi=160)
    plt.close(fig)


def plot_validation_rmse(results_dir: Path) -> None:
    path = results_dir / "validation_metrics.csv"
    if not path.exists():
        return
    df = pd.read_csv(path)
    joint_number = pd.to_numeric(df["joint"], errors="coerce")
    joint_df = df.loc[joint_number.notna()].copy()
    joints = joint_number.loc[joint_number.notna()].astype(int).to_numpy()
    width = 0.36
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(joints - width / 2, joint_df["identified_rmse_nm"], width=width, label="identified")
    ax.bar(joints + width / 2, joint_df["nominal_rmse_nm"], width=width, label="nominal")
    ax.set_xticks(joints)
    ax.set_xlabel("joint")
    ax.set_ylabel("torque RMSE [N m]")
    ax.set_title("Unseen-trajectory torque prediction — absolute error")
    ax.grid(True, axis="y", alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results_dir / "validation_rmse_nm.png", dpi=160)
    plt.close(fig)


def print_acceptance(results_dir: Path) -> None:
    path = results_dir / "acceptance.csv"
    if not path.exists():
        return
    df = pd.read_csv(path)
    print("Acceptance summary:")
    for _, row in df.iterrows():
        print(f"  {row['criterion']}: {row['pass']}  metric={row['metric']}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path, nargs="?", default=Path("results"))
    args = parser.parse_args()
    if not args.results.exists():
        raise FileNotFoundError(f"Results directory does not exist: {args.results}")

    for csv_path in sorted(args.results.glob("trace_*.csv")):
        plot_trace(csv_path, args.results)
    plot_control_rmse(args.results)
    plot_singular_values(args.results)
    plot_validation_nrmse(args.results)
    plot_validation_rmse(args.results)
    print_acceptance(args.results)


if __name__ == "__main__":
    main()
