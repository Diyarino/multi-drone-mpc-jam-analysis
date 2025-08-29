# -*- coding: utf-8 -*-
"""
Created on Tue Jul 15 17:00:55 2025

@author: Altinses
"""

# %%

import numpy as np

# %%

def plot_safety_zones(ax, drones, colors=None, alpha=0.2, resolution=None):
    """
    Draws the safety zones of multiple drones as 3D wireframe spheres on a given Matplotlib axis.

    This function iterates through a list of drone objects, extracts their current
    position and safety radius, and plots a wireframe sphere representing their
    safety zone.

    Parameters
    ----------
    ax : matplotlib.axes._subplots.Axes3DSubplot
        The 3D axes object on which to draw the safety zones. This should be
        created using `fig.add_subplot(111, projection='3d')`.
    drones : list of Drone objects
        A list of `Drone` objects (or any object with `state` (first 3 elements
        as position) and `safety_radius` attributes).
    colors : list of str or None, optional
        A list of Matplotlib color strings to use for each drone's safety zone.
        If None, a default sequence of 'b', 'r', 'g' will be used. The list
        should have at least as many colors as there are drones.
    alpha : float, optional
        The transparency level (0.0 to 1.0) for the wireframe spheres.
        A lower value makes the spheres more transparent. Default is 0.2.
    resolution : int or None, optional
        Determines the resolution of the spherical mesh. A higher value results
        in a smoother sphere but takes more computational time.
        Specifically, it defines the number of points for `u` (azimuthal)
        and `v` (polar) angles. If None, default values of 20j and 10j
        (20 points for u, 10 points for v) will be used.

    Returns
    -------
    None
        The function directly modifies the provided `ax` object by adding the
        wireframe plots.

    See Also
    --------
    matplotlib.axes.Axes.plot_wireframe : Used to plot the 3D wireframes.

    Notes
    -----
    The `safety_radius` of each drone is assumed to be a single numerical value.
    If variable safety radii (e.g., time-varying or anisotropic) are needed,
    this function would require modification to accept and interpret such data.
    """
    if colors is None:
        # Default colors if none are provided
        default_colors = ["royalblue", "red", "green", "black", "mediumslateblue", "orange", "darkviolet",
                       "darkgoldenrod", "silver", "deepskyblue"]
        colors_to_use = default_colors[:len(drones)] # Use enough colors for the drones
        # If there are more drones than default colors, cycle through them
        if len(drones) > len(default_colors):
            colors_to_use = [default_colors[i % len(default_colors)] for i in range(len(drones))]
    else:
        if len(colors) < len(drones):
            print("Warning: Not enough colors provided for all drones. Cycling through provided colors.")
            colors_to_use = [colors[i % len(colors)] for i in range(len(drones))]
        else:
            colors_to_use = colors

    # Set sphere resolution
    if resolution is None:
        u_res = 20j # 20 points
        v_res = 10j # 10 points
    else:
        u_res = complex(0, resolution)
        v_res = complex(0, resolution // 2) # Half resolution for v typically looks good

    # Generate mesh for a unit sphere once
    u, v = np.mgrid[0:2*np.pi:u_res, 0:np.pi:v_res]
    cos_u = np.cos(u)
    sin_u = np.sin(u)
    sin_v = np.sin(v)
    cos_v = np.cos(v)

    for i, drone in enumerate(drones):
        center = drone.state[:3]
        radius = drone.safety_radius
        color = colors_to_use[i]

        # Calculate sphere coordinates by scaling and shifting the unit sphere mesh
        x = center[0] + radius * cos_u * sin_v
        y = center[1] + radius * sin_u * sin_v
        z = center[2] + radius * cos_v

        ax.plot_wireframe(x, y, z, color=color, alpha=alpha)


# %% test

if __name__ == "__main__":
    x = 0