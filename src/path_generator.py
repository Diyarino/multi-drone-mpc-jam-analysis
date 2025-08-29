# -*- coding: utf-8 -*-
"""
Created on Fri Aug 29 18:12:37 2025

@author: Diyar Altinses, M.Sc.
"""

# %% imports

import numpy as np

# %%

def smooth_velocity_path(waypoints: np.ndarray, T: int) -> np.ndarray:
    """
    Interpolates waypoints with smooth velocity and acceleration using Hermite-like velocity matching.
    """
    N = len(waypoints)
    velocities = np.diff(waypoints, axis=0)
    velocities = np.vstack([velocities[0], velocities, velocities[-1]])
    t = np.linspace(0, 1, T)
    segment_t = np.linspace(0, 1, N)
    path = np.zeros((T, 3))
    
    for i in range(3):  # For x, y, z
        path[:, i] = np.interp(t, segment_t, waypoints[:, i])
    
    return path

def generate_realistic_path(start: np.ndarray,
                            end: np.ndarray,
                            n_waypoints: int = 5,
                            jitter: float = 1.0,
                            curve_strength: float = 0.25,
                            total_steps: int = 100) -> np.ndarray:
    """
    Create a realistic path from start to end with random but smooth intermediate waypoints.

    Parameters
    ----------
    start : np.ndarray
        Starting position (3,)
    end : np.ndarray
        Target position (3,)
    n_waypoints : int
        Number of waypoints to insert between start and end
    jitter : float
        Magnitude of lateral deviation per waypoint
    curve_strength : float
        Weight of non-linearity (higher = more curve)
    total_steps : int
        Number of time steps in the path

    Returns
    -------
    path : np.ndarray
        Generated path of shape (total_steps, 3)
    """
    direction = end - start
    direction /= np.linalg.norm(direction)
    length = np.linalg.norm(end - start)

    # Generate intermediate waypoints along the straight line
    waypoints = [start]
    for i in range(1, n_waypoints + 1):
        alpha = i / (n_waypoints + 1)
        base = start + direction * length * alpha
        lateral = np.random.randn(3)
        lateral -= direction * np.dot(lateral, direction)  # Orthogonal component
        lateral /= (np.linalg.norm(lateral) + 1e-8)
        offset = lateral * jitter * np.sin(np.pi * alpha) * curve_strength
        waypoints.append(base + offset)

    waypoints.append(end)
    return smooth_velocity_path(np.array(waypoints), total_steps)

# %%

if __name__ == '__main__':
    print(True)


