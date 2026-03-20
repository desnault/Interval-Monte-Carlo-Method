"""
File: result.py

Author: Damien ESNAULT (PhD student)
Institution: ENSTA/Lab-STICC
Date: 2025

Summary:
    Load, analyze, and display the results of the OCEANS 2025 interval
    Monte Carlo scenario.

Description:
    This script is intended to be run after the C++ script `oceans2025/main.cpp`.
    It loads:
        - the exported probability-bound evolution files,
        - the trajectory classification file,
        - the reference trajectory,
        - the noisy trajectories,

    and produces:
        1) a 2x2 figure showing the evolution of the 4 interval probability bounds,
        2) a second figure showing the reference trajectory, the noisy trajectories
           colored by global mission result, and the three object boxes.

Default path convention:
    If no path is provided by the user, the script assumes the following repo layout:

        repo/
        ├── data/
        │   └── oceans2025/
        │       ├── Perfect/
        │       ├── Noisy/
        │       ├── prob_box1.txt
        │       ├── prob_box2.txt
        │       ├── prob_box3.txt
        │       ├── prob_all.txt
        │       └── trajectory_results.txt
        └── oceans2025/
            └── result.py

Typical use:
    From the repo root:
        python3 oceans2025/result.py

    Or with an explicit data directory:
        python3 oceans2025/result.py --data-path /path/to/repo/data/oceans2025
"""

# =============================================================================
# IMPORT LIBRARIES
# =============================================================================

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


# =============================================================================
# PARSE USER ARGUMENTS
# =============================================================================

# Path to the directory containing this script
script_dir = Path(__file__).resolve().parent

# Default path to the exported scenario data
default_data_path = script_dir.parent / "data" / "oceans2025"

# Parse optional user arguments
parser = argparse.ArgumentParser()
parser.add_argument("--data-path", type=str, default=str(default_data_path))

args = parser.parse_args()

# Path to the exported result directory
data_path = Path(args.data_path)


# =============================================================================
# SCENARIO PARAMETERS
# =============================================================================

# Object boxes used in the OCEANS 2025 scenario
box1 = np.array([[9.5, 10.5], [1.2, 2.2]], dtype=float)
box2 = np.array([[23.0, 24.0], [11.5, 12.5]], dtype=float)
box3 = np.array([[43.0, 44.0], [28.0, 29.0]], dtype=float)

# Exported probability evolution files
prob_box1_filename = data_path / "prob_box1.txt"
prob_box2_filename = data_path / "prob_box2.txt"
prob_box3_filename = data_path / "prob_box3.txt"
prob_all_filename = data_path / "prob_all.txt"

# Exported trajectory classification file
trajectory_results_filename = data_path / "trajectory_results.txt"

# Exported trajectory directories/files
perfect_trajectory_filename = data_path / "Perfect" / "simulation_numpy_array.txt"
noisy_trajectory_directory = data_path / "Noisy"


# =============================================================================
# CHECK PATHS AND FILES
# =============================================================================

# Check that the main data path exists and is a directory
if not data_path.exists():
    raise FileNotFoundError(f"Data directory does not exist: {data_path}")

if not data_path.is_dir():
    raise NotADirectoryError(f"Data path is not a directory: {data_path}")

# Check probability evolution files
if not prob_box1_filename.exists():
    raise FileNotFoundError(f"Missing file: {prob_box1_filename}")

if not prob_box2_filename.exists():
    raise FileNotFoundError(f"Missing file: {prob_box2_filename}")

if not prob_box3_filename.exists():
    raise FileNotFoundError(f"Missing file: {prob_box3_filename}")

if not prob_all_filename.exists():
    raise FileNotFoundError(f"Missing file: {prob_all_filename}")

# Check trajectory classification file
if not trajectory_results_filename.exists():
    raise FileNotFoundError(f"Missing file: {trajectory_results_filename}")

# Check reference trajectory file
if not perfect_trajectory_filename.exists():
    raise FileNotFoundError(f"Missing file: {perfect_trajectory_filename}")

# Check noisy trajectory directory
if not noisy_trajectory_directory.exists():
    raise FileNotFoundError(f"Missing directory: {noisy_trajectory_directory}")

if not noisy_trajectory_directory.is_dir():
    raise NotADirectoryError(f"Noisy trajectory path is not a directory: {noisy_trajectory_directory}")


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def load_probability_file(file_path: Path):
    """
    Load a probability evolution file exported by the C++ script.

    Expected file format:
        N [lower_bound,upper_bound]

    Example:
        1 [1.000000,1.000000]
        2 [1.000000,1.000001]

    Returns:
        n_samples (np.ndarray): Number of processed samples
        lower_bounds (np.ndarray): Lower bound of probability interval
        upper_bounds (np.ndarray): Upper bound of probability interval
    """

    n_samples = []
    lower_bounds = []
    upper_bounds = []

    with open(file_path, "r", encoding="utf-8") as file:
        for line in file:
            stripped_line = line.strip()

            # Skip empty lines
            if stripped_line == "":
                continue

            parts = stripped_line.split()
            if len(parts) != 2:
                raise ValueError(f"Invalid line in probability file: {line}")

            # Parse number of samples
            n_value = float(parts[0])

            # Parse interval [lb, ub]
            interval_text = parts[1]
            if not (interval_text.startswith("[") and interval_text.endswith("]")):
                raise ValueError(f"Invalid interval format in probability file: {line}")

            interval_text = interval_text[1:-1]
            bounds = interval_text.split(",")

            if len(bounds) != 2:
                raise ValueError(f"Invalid interval bounds in probability file: {line}")

            lower_bound = float(bounds[0])
            upper_bound = float(bounds[1])

            n_samples.append(n_value)
            lower_bounds.append(lower_bound)
            upper_bounds.append(upper_bound)

    return (
        np.array(n_samples, dtype=float),
        np.array(lower_bounds, dtype=float),
        np.array(upper_bounds, dtype=float)
    )


def load_trajectory_results(file_path: Path):
    """
    Load trajectory classification results.

    Expected file format:
        filename RESULT

    Example:
        simulation1.tubevector TRUE
        simulation2.tubevector UNKNOWN

    Returns:
        dict[str, str]: Mapping filename -> classification result
    """

    results = {}

    with open(file_path, "r", encoding="utf-8") as file:
        for line in file:
            stripped_line = line.strip()

            # Skip empty lines
            if stripped_line == "":
                continue

            parts = stripped_line.split()
            if len(parts) != 2:
                raise ValueError(f"Invalid line in trajectory results file: {line}")

            filename, result = parts
            results[filename] = result

    return results


def tubevector_filename_to_numpy_filename(tubevector_filename: str):
    """
    Convert a TubeVector filename into the associated numpy trajectory filename.

    Example:
        simulation1.tubevector -> simulation1_numpy_array.txt
    """
    path = Path(tubevector_filename)
    stem = path.stem
    return f"{stem}_numpy_array.txt"


def load_trajectory(file_path: Path):
    """
    Load a trajectory stored as a numpy text file.

    Returns:
        np.ndarray with shape (N, 5):
            columns = [t, x, y, theta, v]
    """
    trajectory = np.loadtxt(file_path, dtype=float)

    # Ensure 2D shape (even for single-line files)
    if trajectory.ndim == 1:
        trajectory = trajectory[np.newaxis, :]

    return trajectory


def box_center(box: np.ndarray):
    """
    Compute the center of a 2D interval box.

    Args:
        box: shape (2,2) array

    Returns:
        np.ndarray: [x_center, y_center]
    """
    return np.array([
        0.5 * (box[0, 0] + box[0, 1]),
        0.5 * (box[1, 0] + box[1, 1])
    ], dtype=float) 


# =============================================================================
# LOAD DATA
# =============================================================================

# Load probability evolution files
n_box1, lb_box1, ub_box1 = load_probability_file(prob_box1_filename)
n_box2, lb_box2, ub_box2 = load_probability_file(prob_box2_filename)
n_box3, lb_box3, ub_box3 = load_probability_file(prob_box3_filename)
n_all, lb_all, ub_all = load_probability_file(prob_all_filename)

# Load trajectory classification results
trajectory_results = load_trajectory_results(trajectory_results_filename)

# Load reference trajectory
perfect_trajectory = load_trajectory(perfect_trajectory_filename)

# Containers for classified noisy trajectories
true_trajectories = []
unknown_trajectories = []
false_trajectories = []

# Load each noisy trajectory and classify it according to the exported result
for tubevector_filename, result in trajectory_results.items():
    numpy_filename = tubevector_filename_to_numpy_filename(tubevector_filename)
    trajectory_path = noisy_trajectory_directory / numpy_filename

    if not trajectory_path.exists():
        raise FileNotFoundError(f"Missing noisy trajectory file: {trajectory_path}")

    trajectory = load_trajectory(trajectory_path)

    if result == "TRUE":
        true_trajectories.append(trajectory)
    elif result == "UNKNOWN":
        unknown_trajectories.append(trajectory)
    elif result == "FALSE":
        false_trajectories.append(trajectory)
    else:
        raise ValueError(f"Unknown trajectory classification result: {result}")


# =============================================================================
# TERMINAL SUMMARY
# =============================================================================

# Final interval probability bounds
final_interval_box1 = (lb_box1[-1], ub_box1[-1])
final_interval_box2 = (lb_box2[-1], ub_box2[-1])
final_interval_box3 = (lb_box3[-1], ub_box3[-1])
final_interval_all = (lb_all[-1], ub_all[-1])

# Number of trajectories in each category
n_true = len(true_trajectories)
n_unknown = len(unknown_trajectories)
n_false = len(false_trajectories)

# Display final numerical results
print()
print("[RESULT] Final interval probability for box 1: [{:.6f}, {:.6f}]".format(final_interval_box1[0], final_interval_box1[1]))
print("[RESULT] Final interval probability for box 2: [{:.6f}, {:.6f}]".format(final_interval_box2[0], final_interval_box2[1]))
print("[RESULT] Final interval probability for box 3: [{:.6f}, {:.6f}]".format(final_interval_box3[0], final_interval_box3[1]))
print("[RESULT] Final interval probability for all boxes: [{:.6f}, {:.6f}]".format(final_interval_all[0], final_interval_all[1]))
print()
print("[RESULT] Number of TRUE trajectories: {}".format(n_true))
print("[RESULT] Number of UNKNOWN trajectories: {}".format(n_unknown))
print("[RESULT] Number of FALSE trajectories: {}".format(n_false))
print()


# =============================================================================
# PROBABILITY EVOLUTION FIGURE
# =============================================================================

# Create a 2x2 figure: one subplot for each probability bound
fig_prob, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False, sharey=False)

# Plot definition:
#   axis, x-values, lower bounds, upper bounds, title
plot_definitions = [
    (axes[0, 0], n_box1, lb_box1, ub_box1, "Box 1"),
    (axes[0, 1], n_box2, lb_box2, ub_box2, "Box 2"),
    (axes[1, 0], n_box3, lb_box3, ub_box3, "Box 3"),
    (axes[1, 1], n_all,  lb_all,  ub_all,  "All boxes"),
]

for ax, n_values, lower_bounds, upper_bounds, title in plot_definitions:
    # Lower and upper bound curves
    ax.plot(n_values, lower_bounds, color="tab:blue", linewidth=2, label="Lower bound")
    ax.plot(n_values, upper_bounds, color="tab:orange", linewidth=2, label="Upper bound")

    # Filled area representing the interval width
    ax.fill_between(n_values, lower_bounds, upper_bounds, color="gold", alpha=0.35, label="Interval width")

    # Highlight one interval in the middle of the computation
    mid_idx = (len(n_values) - 1) // 2
    mid_n = n_values[mid_idx]
    lb = lower_bounds[mid_idx]
    ub = upper_bounds[mid_idx]

    # Draw the interval as a vertical black segment
    ax.plot([mid_n, mid_n], [lb, ub], color="black", linewidth=3, zorder=10)

    # Add the interval value next to the segment
    text_offset = 0.02 * (n_values[-1] - n_values[0])
    ax.text(
        mid_n + text_offset,
        0.5 * (lb + ub),
        f"N={mid_n}\n[{lb:.3f}, {ub:.3f}]",
        fontsize=10,
        fontweight="bold",
        va="center",
        ha="left"
    )

    # Axis formatting
    ax.set_title(title)
    ax.set_xlabel("Number of trajectories")
    ax.set_ylabel("Probability bound")
    ax.set_ylim(0.0, 1.0)
    ax.grid(True)

# Global figure title and legend
handles, labels = axes[0, 0].get_legend_handles_labels()

fig_prob.suptitle("Evolution of interval probability bounds", fontsize=14)

fig_prob.legend(
    handles,
    labels,
    loc="upper center",
    bbox_to_anchor=(0.5, 0.96),
    ncol=3
)

fig_prob.tight_layout()


# =============================================================================
# TRAJECTORY FIGURE
# =============================================================================

# Create figure
fig_traj, ax_traj = plt.subplots(figsize=(12, 8))

# Plot noisy trajectories classified by global result
for trajectory in false_trajectories:
    ax_traj.plot(trajectory[:, 1], trajectory[:, 2], color="red", alpha=0.5, zorder=1)

for trajectory in unknown_trajectories:
    ax_traj.plot(trajectory[:, 1], trajectory[:, 2], color="gold", alpha=0.7, zorder=2)

for trajectory in true_trajectories:
    ax_traj.plot(trajectory[:, 1], trajectory[:, 2], color="green", alpha=0.7, zorder=3)

# Plot reference trajectory (on top)
ax_traj.plot(
    perfect_trajectory[:, 1],
    perfect_trajectory[:, 2],
    color="black",
    linewidth=2.5,
    zorder=20,
    label="Reference trajectory"
)

# Define boxes and associated final probability bounds
box_definitions = [
    (box1, "Box 1", final_interval_box1),
    (box2, "Box 2", final_interval_box2),
    (box3, "Box 3", final_interval_box3),
]

# Draw boxes and annotate with probability bounds
for box, label, interval_value in box_definitions:
    x_min, x_max = box[0, 0], box[0, 1]
    y_min, y_max = box[1, 0], box[1, 1]

    rectangle = patches.Rectangle(
        (x_min, y_min),
        x_max - x_min,
        y_max - y_min,
        linewidth=1.5,
        edgecolor="black",
        facecolor="saddlebrown",
        alpha=1.0,
        zorder=10
    )
    ax_traj.add_patch(rectangle)

    center = box_center(box)

    ax_traj.text(
        center[0],
        y_max + 0.8,
        f"{label}\n[{interval_value[0]:.3f}, {interval_value[1]:.3f}]",
        fontsize=11,
        fontweight="bold",
        ha="center",
        va="bottom",
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
        zorder=30
    )

# Global probability annotation
global_text = (
    f"Global probability bound (all boxes): "
    f"[{final_interval_all[0]:.3f}, {final_interval_all[1]:.3f}]\n"
    f"N = {int(n_all[-1])}"
)

ax_traj.text(
    0.02,
    0.98,
    global_text,
    transform=ax_traj.transAxes,
    fontsize=12,
    fontweight="bold",
    ha="left",
    va="top",
    bbox=dict(facecolor="white", alpha=0.9, edgecolor="black"),
    zorder=40
)

# Legend (proxy artists for categories)
false_proxy, = ax_traj.plot([], [], color="red", label="FALSE", alpha=0.7)
unknown_proxy, = ax_traj.plot([], [], color="gold", label="UNKNOWN", alpha=0.7)
true_proxy, = ax_traj.plot([], [], color="green", label="TRUE", alpha=0.7)

# Axis formatting
ax_traj.set_xlabel("x (m)")
ax_traj.set_ylabel("y (m)")
ax_traj.set_title(
    "Stochastic AUV trajectories classified by three-valued detection outcome",
    fontweight="bold",
    pad=15
)
ax_traj.grid(True)
ax_traj.axis("equal")
ax_traj.legend(loc="upper right", prop={"weight": "bold"})

fig_traj.tight_layout()

# Display figures
plt.show()