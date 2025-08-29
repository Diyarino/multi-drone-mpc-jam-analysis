"""
3D Drone Collision Avoidance with MPC and Safety Zones

Author: Diyar Altinses, M.Sc.
Date: 2023-11-20

To-do:
	- add better docstring and improve the code
"""

#%% imports

import numpy as np
import matplotlib.pyplot as plt

from config_plots import configure_plt
from safety import plot_safety_zones
from mpc import mpc_control
from drone import Drone_restricted as Drone
from path_generator import generate_realistic_path

# %%

class DroneSimulation:
    """
    Handles drone simulation with path forecasting and collision avoidance visualization.
    """

    def __init__(self, T=100, N = 4):
        """
        Initialize the drone simulation.

        Parameters
        ----------
        path1 : torch.Tensor
            Reference path for drone 1.
        path2 : torch.Tensor
            Reference path for drone 2 (or input for scenario model).
        T : int, optional
            Number of simulation steps, by default 100.
        scenario : int, optional
            Forecasting mode (0: none, 1: model-based drone3, 2: passive), by default 0.
        """
        self.N = N
        self.plot_idx = 0
        
        self.T = T
        self.colors = ["royalblue", "red", "green", "black", "mediumslateblue", "orange", "darkviolet",
                       "darkgoldenrod", "silver", "deepskyblue"]
        self.labels = ['Drone '+str(idx+1) for idx in range(N)]
        
        self.paths = self.generate_drone_paths(N=N, T=500, volume=(5, 5, 5), dt=0.1, v_max=5.)
        self.drones = [self._initialize_drone(self.paths[idx], drone_id=idx) for idx in range(N)]
        
        keys = ['drone_' + str(i) for i in range(N)]
        initial_list = [[] for i in range(N)]
        self.costs_bag = dict(zip(keys, initial_list))
        
        for drone, path in zip(self.drones, self.paths):
            drone.ref_path = path

    def _initialize_drone(self, path, drone_id):
        """
        Initialize a drone object with given path.

        Parameters
        ----------
        path : np.ndarray
            Path for the drone.
        drone_id : int
            Unique identifier for the drone.

        Returns
        -------
        Drone
            Initialized drone object.
        """
        initial_vel = (path[1] - path[0]) / 0.3
        return Drone(drone_id, path[0], initial_vel=initial_vel, safety_radius = 1.0)
    
    def generate_drone_paths(self, N, T=100, volume=(5, 5, 5), dt=0.1, v_max=2.0):
        """
        Generate N drone paths inside a 3D volume.
        
        
        Parameters:
        -----------
        N : int
        Number of drones
        T : int
        Number of timesteps per path
        volume : tuple (Lx, Ly, Lz)
        Dimensions of the 3D box
        dt : float
        Time step duration
        v_max : float
        Maximum velocity per axis
        
        
        Returns:
        --------
        paths : np.ndarray
        Shape (N, T, 3), the x,y,z positions of all drones.
        """
        rng = np.random.default_rng(seed=300)
        
        half = volume[0] / 2
        starts = []
        min_dist = 3
        filtered = []
        
        # generate starts with min_dist constraint
        while len(starts) < N:
            p = rng.uniform(-half, half, size=3)
            if all(np.linalg.norm(p - s) >= min_dist for s in starts):
                starts.append(p)
        starts = np.array(starts)
        targets = rng.uniform(-half, half, size=(N, 3))
        
        for i in range(N):
            buffer = generate_realistic_path(start = starts[i], end = targets[i], n_waypoints = 50, jitter = 5.0,
                                curve_strength = 0.5, total_steps = T)
            filtered.append(buffer)
        
        filtered = np.stack(filtered, axis = 0)
        
        return filtered


    def _compute_controls(self, t):
        """
        Compute control inputs for each drone based on scenario mode.

        Parameters
        ----------
        t : int
            Current time step.
        """
        others = [self.drones[:i] + self.drones[i+1:] for i in range(len(self.drones))]
        for idx in range(self.N):
            u, cost = mpc_control(self.drones[idx], others[idx], t, 5)
            self.drones[idx].update_state(u)
            self.costs_bag['drone_'+str(idx)].append(cost)


    def _plot_scene(self, t):
        """
        Plot current state of simulation.

        Parameters
        ----------
        t : int
            Current time step.
        """
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.clear()
        ax.set_xlim(-2.5, 2.5), ax.set_ylim(-2.5, 2.5), ax.set_zlim(-2.5, 2.5)
        ax.set_xlabel('X [m]'), ax.set_ylabel('Y [m]'), ax.set_zlabel('Z [m]')
        ax.view_init(elev=20, azim=20)
        ax.grid(True)

        # Plot reference and predicted paths
        # for idx, path in enumerate(self.paths):
        #     ax.plot(path[:, 0], path[:, 1], path[:, 2], '--', color=self.colors[idx], alpha=0.5)

        # Plot actual paths
        for drone, color, label in zip(self.drones, self.colors, self.labels):
            path = np.array(drone.actual_path)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], '-', color=color, linewidth=2, label=label, alpha=0.3)
            ax.scatter(drone.state[0], drone.state[1], drone.state[2], color=color, s=100, marker='o')
            
        plot_safety_zones(ax, self.drones)
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                drone1 = self.drones[i]
                drone2 = self.drones[j]
                dist = np.linalg.norm(drone1.state[:3] - drone2.state[:3])

                # if dist < 1.99*drone1.safety_radius:
                #     ax.text(2, 0, 0, "SAFETY ZONE VIOLATED! "+str(i+1)+str(j+1), color='red', fontsize=12, fontweight='bold')
        
        check_activity = 0.
        for drone in self.drones:
            if (drone.state == drone.old_state).all():
                check_activity += 1
        if check_activity == self.N:
            ax.text(5, 0, 0, "KINEMATICALLY JAMMED!", color='red', fontsize=18, fontweight='bold')

        # ax.legend(loc='lower right')
        
        plt.tight_layout()
        if t%5 ==0:
            plt.savefig('./figures/buffer2/sample' + str(self.plot_idx) + '.png', bbox_inches='tight', dpi = 150)
            self.plot_idx +=1
            
        plt.show()

    def run(self):
        """
        Run the full simulation loop.
        """
        for t in range(self.T):
            self._compute_controls(t)
            self._plot_scene(t)
            


    # %%

if __name__ == "__main__":
    configure_plt()
    
    # trials_cost = {'drone1': [], 'drone2': []}
    # for i in tqdm(range(30)):

    sim = DroneSimulation(T=500)
    sim.run()
    
    # trials_cost['drone1'].append(sim.drone1_costs_bag)
    # trials_cost['drone2'].append(sim.drone2_costs_bag)
        
    # torch.save(trials_cost, 'costs_dual_priority_gru.pt')

    
    
    
    
    
    