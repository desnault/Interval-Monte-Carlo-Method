# Interval Monte-Carlo Method (IMCM)

The **Interval Monte-Carlo Method (IMCM)** is a framework designed to **bound the probability of success of a system under uncertainty** by combining **Monte Carlo simulation** with **interval analysis** and **three-valued logic**.

Unlike classical Monte Carlo approaches that provide pointwise probability estimates, IMCM produces **interval-valued probability bounds** that explicitly account for:
- uncertainties on system states and environment,
- incomplete knowledge of probability distributions,
- and ambiguity in success/failure classification.

The method is particularly suited for applications where:
- uncertainty is naturally described using **set-membership approaches**,
- system behavior is **stochastic or poorly modeled**,
- and guarantees or conservative bounds are required.

This repository provides:
- a **generic C++ implementation** of the Interval Monte-Carlo framework,
- utilities for **set-based classification using interval analysis**,
- and a complete **illustrative example** applied to an **autonomous underwater vehicle (AUV)** mission.

In this example, the method is used to **bound the probability that an AUV successfully observes multiple uncertain objects** during a mission, highlighting the impact of navigation drift and uncertainty accumulation over time.

> ⚠️ **Recommended reading**
>  
> This README contains both practical examples and theoretical explanations.  
> For a deeper understanding of the method, it is recommended to first read the **"Theory and Concepts"** section before exploring the example scenarios.

> 📖 **Citation**
>
> This work has been presented at **OCEANS 2025 Brest** and published in IEEE Xplore.  
> If you use this repository in your research, please consider citing:
>
> **IEEE format**
> ```
> D. Esnault, S. Rohou, F. Le Bars and L. Jaulin, "Bounding the Success Probability of Naval Mine-Clearance Missions Conducted by AUVs," OCEANS 2025 Brest, BREST, France, 2025, pp. 1-8, doi: 10.1109/OCEANS58557.2025.11104600.
> ```
>
> **BibTeX**
> ```bibtex
> @inproceedings{esnault2025,
>   author    = {Esnault, Damien and Rohou, Simon and Le Bars, Fabrice and Jaulin, Luc},
>   title     = {Bounding the Success Probability of Naval Mine-Clearance Missions Conducted by AUVs},
>   booktitle = {OCEANS 2025 Brest},
>   year      = {2025},
>   pages     = {1--8},
>   doi       = {10.1109/OCEANS58557.2025.11104600}
> }
> ```
>
> 🔗 IEEE Xplore: https://ieeexplore.ieee.org/abstract/document/11104600 <br>
> 🔗 HAL (open access): https://hal.science/hal-05240578

## 📚 Table of Contents

- [⚠️ Project Status](#️-project-status)
- [📦 Dependencies](#-dependencies)
- [🚀 Quick Start](#-quick-start)


- [🌊 OCEANS 2025 Example: Mission Feasibility Analysis](#-oceans-2025-example-mission-feasibility-analysis)
  - [Overview of the Scenario](#overview-of-the-scenario)
  - [Mission Geometry and Uncertainties](#mission-geometry-and-uncertainties)
  - [Three-Valued Logic Interpretation](#three-valued-logic-interpretation)
  - [Results: Probability Bounds Evolution](#results-probability-bounds-evolution)

- [▶️ Reproducing the OCEANS 2025 Results](#️-reproducing-the-oceans-2025-results)
  - [Step 1 — Generate Trajectories](#step-1--generate-trajectories)
  - [Step 2 — (Optional) Visualize Trajectories](#step-2--optional-visualize-trajectories)
  - [Step 3 — Process Trajectories (C++)](#step-3--process-trajectories-c)
  - [Step 4 — Display Results (Python)](#step-4--display-results-python)

- [🗂️ Repository Structure](#️-repository-structure)

- [🧠 Theory and Concepts](#-theory-and-concepts)
  - [Motivation](#motivation)
  - [Three-Valued Monte Carlo Principle](#three-valued-monte-carlo-principle)
  - [Interval-Based Classification](#interval-based-classification)
  - [Empirical Probability Bounds](#empirical-probability-bounds)
  - [Convergence and Interpretation](#convergence-and-interpretation)

- [🔮 Future Work](#-future-work)

- [📚 References](#-references)

## ⚠️ Project Status

This repository provides a **research-oriented implementation** of the Interval Monte-Carlo Method.

- The code is actively used for research and demonstration purposes.
- It is **not intended as a production-ready or fully maintained software**.
- The API and structure may evolve over time without strict backward compatibility.

The current implementation is based on **CODAC v1**.

> 🔧 A future update is planned to ensure compatibility with **CODAC v2**,  
> but no release timeline is defined at this stage.

Users are encouraged to:
- adapt the code to their own needs,
- and use it as a **basis for experimentation and research**.

## 📦 Dependencies

This project relies on the **CODAC ecosystem** and standard scientific Python libraries.

### C++ dependencies

The core implementation is written in C++ and depends on:

- **CODAC** — Contractors, separators, and tubes for set-based analysis  
- **IBEX** — Interval arithmetic library  
- **Eigen3** — Linear algebra library  
- **CMake** ≥ 3.12  
- A C++17 compatible compiler (e.g., `g++`, `clang++`)

> ✅ **Recommended setup**
>
> It is strongly recommended to install **CODAC** by following the official instructions:  
> https://www.codac.io/install/01-installation.html
>
> When CODAC is installed properly:
> - **IBEX** is installed automatically,
> - **Eigen3** is also handled by the installation.
>
> 👉 In that case, no additional installation of IBEX or Eigen is required.

---

### Python dependencies

Python is used for:
- trajectory generation,
- data visualization,
- and result analysis.

The required libraries are:

- `numpy`
- `matplotlib`

These can be installed with:

```bash
pip install numpy matplotlib
```

> ℹ️ The scripts have been developed using a standard Python environment.
>
> You may use either:
> - system Python, or
> - a virtual environment (venv, conda, etc.)

### Notes

The project has been tested with:
- Python ≥ 3.8
- CMake ≥ 3.12

No GPU or special hardware is required.

Performance mainly depends on:
- the number of Monte Carlo samples,
- the resolution used in set-based computations.


## 🚀 Quick Start

This section explains how to **clone, configure, build, and validate** the project with a minimal workflow.

### 1. Clone the repository

Clone the repository **with submodules**, since the project relies on the `Codac-Coverage-Toolbox` submodule:

```bash
git clone --recurse-submodules https://github.com/desnault/Interval-Monte-Carlo-Method.git
cd Interval-Monte-Carlo-Method
```

If you already cloned the repository without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

### 2. Create the recommended `CMakeLists.txt`

To simplify compilation, place the following `CMakeLists.txt` file at the root of the repository:
```cmake
# ==============================================================
# CMakeLists.txt for Interval Monte Carlo Project
# ==============================================================

cmake_minimum_required(VERSION 3.12)
project(interval_monte_carlo_method LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================
# ---------------- Project data directory ---------------------
# ==============================================================

set(INTERVAL_MC_DATA_DIR "${CMAKE_SOURCE_DIR}/data")

# ==============================================================
# ----------- Import external libraries ----------------------
# ==============================================================

# IBEX: library for interval arithmetic
find_package(IBEX REQUIRED)
ibex_init_common()  # initialize IBEX common settings
message(STATUS "Found IBEX version ${IBEX_VERSION}")

# Eigen3: linear algebra library
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Found Eigen3 version ${Eigen3_VERSION}")

# CODAC: library for contractors, separators, tubes, etc.
find_package(CODAC REQUIRED)
message(STATUS "Found CODAC version ${CODAC_VERSION}")

# ==============================================================
# --------- Compile main library (all src/) ------------------
# ==============================================================

# Gather all .h and .cpp files in src/ recursively
file(GLOB_RECURSE SRC_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Create static library from all source files
add_library(interval_project_lib STATIC ${SRC_SOURCES})

# Specify the data directory to all project
target_compile_definitions(interval_project_lib PUBLIC
    INTERVAL_MC_DATA_DIR="${INTERVAL_MC_DATA_DIR}"
)

# Include directories for this library
# - Add root src/ folder
# - Add all immediate subfolders of src/ so examples can do #include "Filename.h" without subfolder prefix
target_include_directories(interval_project_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${IBEX_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CODAC_INCLUDE_DIRS}
)

# Add all immediate subfolders of src/ as include directories
file(GLOB SRC_SUBDIRS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/src/*")
foreach(dir ${SRC_SUBDIRS})
    if(IS_DIRECTORY ${dir})
        target_include_directories(interval_project_lib PUBLIC ${dir})
    endif()
endforeach()

# Apply CODAC-specific compiler options
target_compile_options(interval_project_lib PUBLIC ${CODAC_CXX_FLAGS})

# Link external libraries (CODAC + IBEX)
target_link_libraries(interval_project_lib PUBLIC ${CODAC_LIBRARIES} Ibex::ibex)

# ==============================================================
# ---------------- Compile examples scripts -------------------
# ==============================================================

# Helper function to create executables recursively
function(add_recursive_executables root_dir output_dir)
    # Find all .cpp files recursively in root_dir
    file(GLOB_RECURSE SCRIPT_SOURCES
        CONFIGURE_DEPENDS
        "${root_dir}/*.cpp"
    )

    foreach(script_src IN LISTS SCRIPT_SOURCES)
        # Compute relative path from root_dir
        file(RELATIVE_PATH rel_path "${root_dir}" "${script_src}")

        # Extract filename without extension
        get_filename_component(exec_name ${rel_path} NAME_WE)

        # Extract subdirectory of script relative to root_dir
        get_filename_component(exec_subdir ${rel_path} DIRECTORY)

        # Create target name (to avoid conflicts)
        string(REPLACE "/" "_" target_name "${exec_subdir}_${exec_name}")

        # Create executable
        add_executable(${target_name} ${script_src})

        # Link main library
        target_link_libraries(${target_name} PRIVATE interval_project_lib)

        # Include directories for library headers
        target_include_directories(${target_name} PRIVATE ${CMAKE_SOURCE_DIR}/src)

        # CODAC-specific compiler flags
        target_compile_options(${target_name} PRIVATE ${CODAC_CXX_FLAGS})

        # Include system libraries
        target_include_directories(${target_name} SYSTEM PRIVATE
            ${CODAC_INCLUDE_DIRS}
            ${IBEX_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS}
        )

        # Compute final output directory
        set(final_output_dir "${CMAKE_BINARY_DIR}/${output_dir}/${exec_subdir}")
        file(MAKE_DIRECTORY ${final_output_dir})

        # Set executable properties
        set_target_properties(${target_name} PROPERTIES
            OUTPUT_NAME "${exec_name}"
            RUNTIME_OUTPUT_DIRECTORY "${final_output_dir}"
        )
    endforeach()
endfunction()

# Compile examples/ folder (preserve subfolder structure)
add_recursive_executables("${CMAKE_SOURCE_DIR}/examples" "examples")

# Compile oceans2025/ folder (all scripts directly in build/oceans2025)
add_recursive_executables("${CMAKE_SOURCE_DIR}/oceans2025" "oceans2025")
```

This file:
- compiles all source files located in `src/`,
- makes the `data/` directory visible to the C++ scripts through the `INTERVAL_MC_DATA_DIR` definition,
- compiles the example scripts located in `examples/`,
- and compiles the scripts located in `oceans2025/`.

### 3. Build the project

A standard **out-of-source build** is recommended:
```bash
mkdir build
cd build
cmake ..
make
cd ..
```

### 4. Run a simple example to validate the installation

A good first test is the second `IntervalMonteCarlo` example:
```bash
./examples/IntervalMonteCarlo/example2_custom_struct_IntervalMonteCarlo
```

Expected terminal output:
```
Number of trajectory(ies) loaded: 10

Trajectory 1/10:
	Local evaluation: [0, 1]
Trajectory 2/10:
	Local evaluation: <1, 1>
Trajectory 3/10:
	Local evaluation: [0, 1]
Trajectory 4/10:
	Local evaluation: <1, 1>
Trajectory 5/10:
	Local evaluation: <1, 1>
Trajectory 6/10:
	Local evaluation: <1, 1>
Trajectory 7/10:
	Local evaluation: <1, 1>
Trajectory 8/10:
	Local evaluation: [0, 1]
Trajectory 9/10:
	Local evaluation: <1, 1>
Trajectory 10/10:
	Local evaluation: <1, 1>

The estimated interval probability is: [0.6999999999999999, 1.000000000000001]
```

The script loads the trajectories stored in `data/examples` and evaluates whether a target box is fully covered, not covered, or uncertainly covered along each trajectory.

To visually verify the result of each trajectory classification, you can inspect the images provided in:
```
data/examples
```

These images show the trajectory, the covered area, and the box to cover, allowing you to check that the local evaluations are consistent with the geometry.

> ⏳ **Build and runtime note**
>  
> - Compilation may take some time depending on your machine and your CODAC/IBEX installation.
>
> - Most example scripts execute quickly.
>
> - More advanced scripts, especially those relying on fine paving or large Monte Carlo datasets, may require a few minutes.

## 🌊 OCEANS 2025 Example: Mission Feasibility Analysis

This section illustrates the use of the **Interval Monte-Carlo Method (IMCM)** on a realistic scenario inspired by autonomous underwater robotics.

The objective is to evaluate the **probability of success of an inspection mission** conducted by an autonomous underwater vehicle (AUV), in the presence of:
- **uncertainties on object locations**,  
- and **stochastic deviations in vehicle motion**.

The example demonstrates how IMCM can be used to:
- model uncertain environments using **intervals**,
- classify mission outcomes using **three-valued logic**,
- and compute **interval-valued probability bounds** from simulated trajectories.

> ⚠️ **Recommended reading**
>
> This example is detailed and builds upon key concepts of the method.  
> It is recommended to first read the **"Theory and Concepts"** section to understand:
> - the three-valued logic framework,
> - the interval-based classification,
> - and the interpretation of probability bounds.

The complete workflow associated with this example (trajectory generation, processing, and visualization) is described in the section:
👉 **"Reproducing the OCEANS 2025 Results"**.


### Overview of the Scenario

We consider an **inspection mission conducted by an autonomous underwater vehicle (AUV)** operating at constant depth over a flat seabed.  
The mission objective is to **observe three objects** previously detected during an earlier survey.

The AUV follows a predefined trajectory designed to pass near each object.  
However, due to navigation uncertainty (e.g., dead-reckoning drift), the **actual trajectory deviates from the nominal one**, making mission success uncertain.

The figure below provides a global overview of the scenario:

<p align="center">
  <img src="oceans2025/mission_overlook.png" width="700">
</p>

In this figure:
- The **black curve** represents the nominal (reference) trajectory.
- The **colored curves** represent simulated trajectories affected by stochastic perturbations.
- The **three brown boxes** represent uncertain object locations on the seabed.
- The **circular footprints** correspond to the sensing range of the AUV.

Each object is not represented by a single point but by a **2D bounding box**, reflecting uncertainty on its exact position.  
The AUV is equipped with a **range-based sensor**, and an object is considered detected if it lies within the sensor footprint at some point along the trajectory.

This scenario captures a common situation in marine robotics:
- a mission planned using a nominal model,
- executed under uncertainty,
- where success depends on the interaction between **trajectory deviation** and **environmental uncertainty**.

### Mission Geometry and Uncertainties

The mission is defined in a **2D spatial framework**, assuming constant depth and a flat seabed.  
The state of the AUV is described by its planar position and heading, and its motion is modeled using a **Dubins-like kinematic model**.

#### Object representation

The three target objects are modeled as **uncertain regions** using interval boxes:

\[
\mathcal{B}_i \subset \mathbb{R}^2, \quad i = 1,2,3
\]

Each box represents a set of possible positions where the object may lie.  
No probability distribution is assumed inside the box: the only available information is that the object is **guaranteed to be somewhere within this region**.

#### Sensor model

The AUV is equipped with a **range-based sensor** of fixed radius \( R \).  
At any time \( t \), the sensor footprint is modeled as a disk:

\[
\mathcal{D}(t) = \{ x \in \mathbb{R}^2 \mid \|x - p(t)\| \leq R \}
\]

where \( p(t) \) is the position of the AUV at time \( t \).

An object is considered detected if its entire uncertainty box intersects the sensor footprint at least once along the trajectory.

#### Stochastic trajectory model

Although the mission is designed using a deterministic model, the **true AUV motion is stochastic**.

In this example:
- the control inputs are defined as deterministic time-dependent functions,
- but **noise is added to the heading command during integration**,
- resulting in a set of **random trajectories**.

This models a realistic **dead-reckoning navigation behavior**, where:
- small orientation errors accumulate over time,
- and lead to increasing deviation from the nominal trajectory.

#### Implication for mission success

As time progresses:
- the dispersion of trajectories increases,
- and the probability of successfully observing distant objects decreases.

The mission success is therefore not deterministic, but must be evaluated **statistically**, while accounting for:
- spatial uncertainty (object boxes),
- and trajectory variability (Monte Carlo samples).