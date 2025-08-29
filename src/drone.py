# -*- coding: utf-8 -*-
"""
Created on Tue Jul 15 09:42:16 2025

@author: Diyar Altinses, M.Sc.
"""
# %% imports

import numpy as np

# %%

class Drone:
    """
    A drone agent with dynamics, reference path, and safety zone.
    
    Args:
        id (int): Drone identifier.
        initial_pos (np.ndarray): Initial position [x, y, z].
        initial_vel (np.ndarray): Initial velocity [vx, vy, vz].
        dt (float): Time step.
    """
    def __init__(self, id, initial_pos, initial_vel, dt=0.1, safety_radius = 1.0):
        
        self.id = id
        self.state = np.hstack([initial_pos, initial_vel])
        self.dt = dt
        
        self.A = np.block([
            [np.eye(3), self.dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)]
        ])
        
        self.B = np.block([
            [0.5 * self.dt**2 * np.eye(3)],
            [self.dt * np.eye(3)]
        ]).reshape(6, 3)
        
        self.ref_path = None
        self.actual_path = []
        self.safety_radius = safety_radius  # Safety zone radius
        self.var_safety_radius = False

    def update_state(self, u):
        """Update state and log position."""
        self.state = self.A @ self.state + self.B @ u
        self.actual_path.append(self.state[:3].copy())
        
        
        
        
class Drone_restricted:
    """
    A drone agent with dynamics, reference path, and safety zone.
    
    Args:
        id (int): Drone identifier.
        initial_pos (np.ndarray): Initial position [x, y, z].
        initial_vel (np.ndarray): Initial velocity [vx, vy, vz].
        dt (float): Time step.
    """
    def __init__(self, id, initial_pos, initial_vel, dt=0.1, safety_radius = 1.0):
        
        self.id = id
        self.state = np.hstack([initial_pos, initial_vel])
        self.dt = dt
        
        self.A = np.block([
            [np.eye(3), self.dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)]
        ])
        
        self.B = np.block([
            [0.5 * self.dt**2 * np.eye(3)],
            [self.dt * np.eye(3)]
        ]).reshape(6, 3)
        
        self.ref_path = None
        self.actual_path = []
        self.safety_radius = safety_radius  # Safety zone radius
        
        self.old_state = self.state

    def update_state(self, u):
        """Update state and log position."""
        self.old_state = self.state
        
        if (u == np.zeros(3)).all():
            self.state = self.old_state
        else:
            self.state = self.A @ self.state + self.B @ u
        
        self.state[:3] = np.clip(self.state[:3], -2.5, 2.5)
        self.state[3:] = np.clip(self.state[3:], -1., 1.)
        
        self.actual_path.append(self.state[:3].copy())
            
        return True

# %% test

if __name__ == '__main__':
    drone1 = Drone(1, np.zeros(20), np.zeros(20))