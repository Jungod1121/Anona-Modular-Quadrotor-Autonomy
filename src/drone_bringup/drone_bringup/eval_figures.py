"""Publication-style evaluation figures (Academic Figure Skill baseline).

Style adapted from reference_repos/academic-figure-skill
(https://github.com/TingxiYu/academic-figure-skill): restrained Nature palette,
thin spines, no top/right border, panel letters, 300 dpi export.
"""

from __future__ import annotations

import math
import os
from typing import List, Optional, Sequence, Tuple

import numpy as np

# Academic Figure Skill Nature/Cell/Science Color Palette — COPY VERBATIM
CATEGORICAL = ["#2166AC", "#B2182B", "#1B7837", "#F1A340", "#762A83", "#666666"]
ACCENT_RED = "#B2182B"
GREY = "#999999"
BLACK = "#222222"


def _apply_style() -> None:
    import matplotlib as mpl

    # Typography + export baselines (Academic Figure Skill).
    mpl.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "Liberation Sans", "DejaVu Sans"],
        "font.size": 8,
        "axes.titlesize": 8,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "figure.titlesize": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.6,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "legend.frameon": False,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
        "savefig.bbox": "tight",
        "savefig.dpi": 300,
        "axes.grid": False,
    })


def _panel_label(ax, letter: str) -> None:
    ax.text(
        -0.12, 1.06, letter, transform=ax.transAxes,
        fontsize=9, fontweight="bold", va="top", color=BLACK,
    )


def _light_guides(ax) -> None:
    ax.grid(True, which="major", color="#E6E6E6", linewidth=0.4, alpha=0.9)
    ax.set_axisbelow(True)


def _rolling_nanmean(x: np.ndarray, win: int) -> np.ndarray:
    """Centered rolling mean; edges use available samples."""
    x = np.asarray(x, dtype=float)
    if win <= 1 or x.size == 0:
        return x.copy()
    win = int(win)
    if win % 2 == 0:
        win += 1
    pad = win // 2
    # Cumulative sum trick with nan→0 + counts.
    valid = np.isfinite(x).astype(float)
    vals = np.where(np.isfinite(x), x, 0.0)
    csum = np.concatenate([[0.0], np.cumsum(vals)])
    ccnt = np.concatenate([[0.0], np.cumsum(valid)])
    out = np.full_like(x, np.nan, dtype=float)
    for i in range(x.size):
        lo = max(0, i - pad)
        hi = min(x.size, i + pad + 1)
        n = ccnt[hi] - ccnt[lo]
        if n > 0:
            out[i] = (csum[hi] - csum[lo]) / n
    return out


def save_evaluation_figure(
    output_path: str,
    ts: np.ndarray,
    errs: Sequence[float],
    px: np.ndarray,
    py: np.ndarray,
    rpm: np.ndarray,
    min_obs: Sequence[float],
    goal_xy: Tuple[float, float],
    planned_xy: Optional[np.ndarray] = None,
    err_limit: float = 0.3,
    obs_limit: float = 0.35,
) -> str:
    """Write a 2×2 evaluation figure to ``output_path`` (PNG). Returns path."""
    _apply_style()
    import matplotlib.pyplot as plt

    ts = np.asarray(ts, dtype=float)
    errs = np.asarray(errs, dtype=float)
    px = np.asarray(px, dtype=float)
    py = np.asarray(py, dtype=float)
    rpm = np.asarray(rpm, dtype=float)
    if rpm.ndim != 2 or rpm.shape[1] != 4:
        raise ValueError(f"rpm must be (N,4), got {rpm.shape}")

    mm = 1.0 / 25.4
    # Double-column width (~183 mm), square-ish multipanel.
    fig, axes = plt.subplots(2, 2, figsize=(183 * mm, 140 * mm))
    ax_err, ax_rpm, ax_xy, ax_obs = axes.flat

    # (a) Position error — primary blue; threshold as thin grey/red guide.
    ax_err.plot(ts, errs, color=CATEGORICAL[0], lw=0.9, solid_capstyle="round")
    ax_err.axhline(
        err_limit, color=ACCENT_RED, ls="--", lw=0.7, label=f"{err_limit:g} m limit")
    ax_err.set_xlabel("t [s]")
    ax_err.set_ylabel("Position error [m]")
    ax_err.set_title("Position error", loc="left", fontweight="bold", pad=2)
    ax_err.legend(loc="upper right", handlelength=1.6)
    _light_guides(ax_err)
    _panel_label(ax_err, "a")

    # (b) Motor RPM — smooth then mean + envelope (raw 4-ch overlay is illegible).
    dt = float(np.median(np.diff(ts))) if ts.size > 1 else 0.05
    win = max(5, int(round(1.0 / max(dt, 1e-3))))  # ~1 s rolling window
    rpm_s = np.column_stack([_rolling_nanmean(rpm[:, i], win) for i in range(4)])
    rpm_mean = np.nanmean(rpm_s, axis=1)
    rpm_lo = np.nanmin(rpm_s, axis=1)
    rpm_hi = np.nanmax(rpm_s, axis=1)
    ax_rpm.fill_between(
        ts, rpm_lo, rpm_hi, color=CATEGORICAL[0], alpha=0.22, linewidth=0,
        label="min–max (1 s smooth)",
    )
    ax_rpm.plot(ts, rpm_mean, color=CATEGORICAL[0], lw=1.15, label="mean RPM")
    ax_rpm.set_xlabel("t [s]")
    ax_rpm.set_ylabel("Motor RPM")
    ax_rpm.set_title("Motor RPM", loc="left", fontweight="bold", pad=2)
    ax_rpm.legend(loc="best", handlelength=1.6)
    _light_guides(ax_rpm)
    _panel_label(ax_rpm, "b")

    # (c) XY trajectory — light downsample only for stroke clarity when N is huge.
    n = px.size
    step = max(1, n // 4000)
    ax_xy.plot(
        px[::step], py[::step], color=CATEGORICAL[0], lw=1.0, label="flown",
        solid_capstyle="round", zorder=3,
    )
    if planned_xy is not None and len(planned_xy) > 1:
        ax_xy.plot(
            planned_xy[:, 0], planned_xy[:, 1],
            color=CATEGORICAL[3], ls="--", lw=0.9, alpha=0.95,
            label="planned", zorder=2,
        )
    ax_xy.scatter(
        [px[0]], [py[0]], s=22, c=CATEGORICAL[2], marker="o",
        zorder=4, label="start", edgecolors="white", linewidths=0.4,
    )
    ax_xy.scatter(
        [goal_xy[0]], [goal_xy[1]], s=48, c=ACCENT_RED, marker="*",
        zorder=5, label="goal",
    )
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY trajectory", loc="left", fontweight="bold", pad=2)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best", handlelength=1.6, ncol=1)
    _light_guides(ax_xy)
    _panel_label(ax_xy, "c")

    # (d) Min obstacle distance
    obs = np.asarray(min_obs, dtype=float)
    finite = np.isfinite(obs)
    if np.any(finite):
        ax_obs.plot(
            ts[finite], obs[finite], color=CATEGORICAL[0], lw=0.9,
            solid_capstyle="round",
        )
        ax_obs.axhline(
            obs_limit, color=ACCENT_RED, ls="--", lw=0.7,
            label=f"safety {obs_limit:g} m",
        )
        ax_obs.set_ylabel("Min obstacle dist. [m]")
        ax_obs.legend(loc="best", handlelength=1.6)
        _light_guides(ax_obs)
    else:
        ax_obs.text(
            0.5, 0.5, "No obstacle cloud", ha="center", va="center",
            transform=ax_obs.transAxes, color=GREY, fontsize=8,
        )
        ax_obs.set_xticks([])
        ax_obs.set_yticks([])
        for spine in ax_obs.spines.values():
            spine.set_visible(False)
    ax_obs.set_xlabel("t [s]")
    ax_obs.set_title("Min obstacle distance", loc="left", fontweight="bold", pad=2)
    _panel_label(ax_obs, "d")

    fig.tight_layout(w_pad=1.2, h_pad=1.2)
    out = os.path.abspath(output_path)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    fig.savefig(out, dpi=300, bbox_inches="tight")
    # Vector sibling for archival / print.
    pdf = os.path.splitext(out)[0] + ".pdf"
    try:
        fig.savefig(pdf, dpi=300, bbox_inches="tight")
    except Exception:
        pass
    plt.close(fig)
    return out


def save_evaluation_figure_from_csv(
    metrics_csv: str,
    output_path: str,
    goal_xy: Tuple[float, float] = (0.0, 0.0),
    planned_path_csv: Optional[str] = None,
) -> str:
    """Rebuild evaluation.png from exported acceptance CSVs."""
    import csv as _csv

    ts: List[float] = []
    errs: List[float] = []
    px: List[float] = []
    py: List[float] = []
    rpm_rows: List[List[float]] = []
    mins: List[float] = []
    with open(metrics_csv, newline="") as f:
        for row in _csv.DictReader(f):
            ts.append(float(row["t"]))
            errs.append(float(row["pos_err"]))
            px.append(float(row["px"]))
            py.append(float(row["py"]))
            rpm_rows.append([
                float(row["rpm0"]), float(row["rpm1"]),
                float(row["rpm2"]), float(row["rpm3"]),
            ])
            raw = row.get("min_obstacle_dist", "inf")
            mins.append(float("inf") if raw in ("inf", "nan", "") else float(raw))

    planned = None
    if planned_path_csv and os.path.isfile(planned_path_csv):
        pts = []
        with open(planned_path_csv, newline="") as f:
            for row in _csv.DictReader(f):
                pts.append([float(row["px"]), float(row["py"])])
        if pts:
            planned = np.asarray(pts, dtype=float)

    return save_evaluation_figure(
        output_path=output_path,
        ts=np.asarray(ts),
        errs=errs,
        px=np.asarray(px),
        py=np.asarray(py),
        rpm=np.asarray(rpm_rows),
        min_obs=mins,
        goal_xy=goal_xy,
        planned_xy=planned,
    )
