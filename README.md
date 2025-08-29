
# Multi-Drone MPC Feasibility and Jam Analysis

This repository contains code, mathematical derivations, and simulations for studying **feasibility limits of Model Predictive Control (MPC) in multi-drone systems** under safety, dynamic, and geometric constraints. The project combines theoretical bounds with empirical validation through randomized trajectory simulations.

---

## 📖 Overview

Autonomous multi-drone systems face fundamental limits in density, coordination, and control feasibility. This project explores these limits by combining:

* **Analytical results**:

  * Safety packing limits and covering-based jam thresholds.
  * Dynamic exclusion zones due to bounded velocity and acceleration.
  * Cut–set (throughput) bounds and max-flow/min-cut analogues.
  * Horizon-based infeasibility conditions using conflict graphs.

* **Simulation experiments**:

  * Random trajectory generation within a 3D domain.
  * MPC-based collision avoidance.
  * Observation of phase transitions from smooth operation to blocking/jamming.

---

## 📂 Repository Structure

```
├── docs/                 # LaTeX sources for the paper/manuscript  
├── src/                  # Core Python simulation code
│   ├── drone.py          # Drone dynamics and MPC setup  
│   ├── safety.py         # Safety zone plotting  
│   ├── config_plots.py   # Plot configurations  
│   └── path_generator.py # Create randomized path
├── results/              # Figures and logs from experiments  
├── README.md             # This file
├── main.py               # Main simulation 
└── requirements.txt      # Python dependencies  
```

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/diyarino/multi-drone-mpc-jam-analysis.git
cd multi-drone-mpc
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```
---

## 🔬 Key Results

* **Analytical bounds** predict that with the chosen parameters, at most **six drones** can be accommodated before dynamic blocking is guaranteed in the worst case.
* **Simulation with 4 drones**: all agents remain feasible, avoiding collisions and reaching their goals.
* **Simulation with 8 drones**: drones become blocked, exhibit oscillatory jittering, and cannot follow their trajectories.

These results illustrate the gap between **worst-case theoretical guarantees** and **typical-case empirical performance**.



---

## 📊 Figures

<p align="center">
  <img src="results//animation_4.gif" width="400" height="400" alt="four" style="margin-right: 10px;">
  <img src="results//animation_8.gif" width="400" height="400" alt="eight Control" style="margin-right: 10px;">
</p>

---

## 🛠️ Requirements

* Python ≥ 3.9
* NumPy
* SciPy
* Matplotlib
* (Optional) JAX or CasADi for MPC optimization

---

## 📌 Citation

If you use this work in your research, please cite:

```
@article{your_citation_key,
  title={Feasibility Limits of Multi-Drone MPC: Safety, Dynamics, and Jamming},
  author={Diyar Altinses},
  journal={arXiv preprint arXiv:XXXX.XXXXX},
  year={2025}
}
```

---



## 📚 References 

Below are selected related works and projects that inspired or complement this research:

<a id="1">[1]</a> Altinses, D., Torres, D. O. S., Lier, S., & Schwung, A. (2025, February). Neural Data Fusion Enhanced PD Control for Precision Drone Landing in Synthetic Environments. In 2025 IEEE International Conference on Mechatronics (ICM) (pp. 1-7). IEEE.

<a id="1">[2]</a> Altinses, D., Salazar Torres, D. O., Schwung, M., Lier, S., & Schwung, A. (2024). Optimizing Drone Logistics: A Scoring Algorithm for Enhanced Decision Making across Diverse Domains in Drone Airlines. Drones, 8(7), 307.

<a id="1">[3]</a> Altinses, D., Torres, D. O. S., Gobachew, A. M., Lier, S., & Schwung, A. (2024). Synthetic Dataset Generation for Optimizing Multimodal Drone Delivery Systems. Drones (2504-446X), 8(12).


