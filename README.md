# MuJoCo-Sim

A MuJoCo-based simulation for testing inverse kinematics (IK) on simplified and target robot leg models, including an approximation of the Cassie robot leg.

---

## 📁 Project Overview

This project currently includes:

* A basic graphics check.
* An inverse kinematics solver for a simplified 2-joint leg.
* A preliminary (unstable) IK setup for the Cassie robot leg using MuJoCo.

---

## 🛠️ Prerequisites

* [MuJoCo 3.3.2](https://mujoco.org/) (set up on your system)
* GCC (for compiling the simple IK solver)
* A C++ compiler if integrating your own modules with MuJoCo
* Windows OS (tested on)

---

## 🚀 Getting Started

### ✅ 1. Run a basic graphics check

```bash
make graphics_check
```

This launches a simple graphics test to verify your MuJoCo visualization is working correctly.

---

### ⚙️ 2. Run Cassie IK Simulation (currently unstable)

Run the MuJoCo simulation using the Cassie model with restricted joints (hip and knee only):

```bash
make cassie_run model="C:/Users/navde/OneDrive/Documents/dev/mujoco-3.3.2-windows-x86_64/model/cassie/cassiepole_x.xml"
```

> ⚠️ *Note:* The IK for this model is still under development and may be unstable.

---

### 🧠 3. Compile and Run Simplified IK Solver

This module provides a standalone inverse kinematics solver for a basic 2-link leg. It serves as an approximation and development baseline.

```bash
gcc ik_leg.cc -o ik_leg -lm
./ik_leg
```

### 🧠 4. Compile and Run Simplified 5-DOF IK Solver

This is approximate sol. in testing

```bash
make ik_bi_run
```

---

## 🤖 Model Details

* **Simple Leg IK**: 2-joint leg (hip + knee) in 2D or 3D space.
* **Cassie Approximation**: Reduced DoF version focusing on hip and knee joints.

---

## 📌 Notes

* Ensure the paths in `make` commands are updated based on your local file system.
* The simplified IK model can help debug joint trajectories and convergence behavior before applying them to the full Cassie model.

---

## 🛍️ Future Improvements

* Stabilize Cassie leg IK convergence
* Add foot orientation control
* Integrate real-time footstep planning

