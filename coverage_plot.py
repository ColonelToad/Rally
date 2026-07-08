import matplotlib
matplotlib.use('Agg')

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_coverage():
    print("Loading workspace data...")
    # 1. Load the C++ generated point cloud
    try:
        df = pd.read_csv("build/workspace.csv")
    except FileNotFoundError:
        print("Error: Could not find build/workspace.csv. Make sure you ran ./workspace")
        return

    # 2. Downsample for rendering performance
    downsample_factor = 50 
    df_sampled = df.iloc[::downsample_factor, :]
    
    # --- BASE PLACEMENT OFFSETS (in meters) ---
    # Tweak these numbers to slide the blue cloud around!
    # Positive X moves the robot forward (closer to the net).
    # Positive Y moves the robot to the left.
    base_offset_x = 0.6  # Starting guess: 60cm forward
    base_offset_y = 0.4  # Starting guess: 40cm offset to the side
    base_offset_z = 0.0  # Keep table height the same
    
    x = df_sampled['x'].values + base_offset_x
    y = df_sampled['y'].values + base_offset_y
    z = df_sampled['z'].values + base_offset_z

    # 3. Generate dataset ball trajectories (Placeholder)
    # Replace this block with your actual DeepMind dataset parser later.
    # For now, we use a slightly expanded version of your rollout trajectory 
    # to represent the "net and table center" cluster you mentioned.
    steps = 500
    t = np.linspace(0, np.pi, steps)
    ball_x = 0.5 + 0.3 * np.cos(t)  # Pushing further out in X
    ball_y = 0.4 * np.sin(t)        # Spreading across Y
    ball_z = 0.5 + 0.2 * np.sin(t)  # Bouncing height in Z

    # 4. Setup 3D Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    print("Rendering plot...")
    
    # Plot Workspace (Low opacity, small markers to look like a volumetric cloud)
    ax.scatter(x, y, z, c='royalblue', s=2, alpha=0.4, label='Panda Reachable Workspace')

    # Plot Ball Trajectories (High opacity, distinct color)
    ax.scatter(ball_x, ball_y, ball_z, c='red', s=5, alpha=1.0, label='Ball Dataset')

    # 5. Format the view to match MuJoCo's coordinate system
    ax.set_xlabel('X (meters) - Forward')
    ax.set_ylabel('Y (meters) - Left/Right')
    ax.set_zlabel('Z (meters) - Up')
    ax.set_title('Franka Panda Workspace vs Ball Trajectories')
    
    # Force equal aspect ratio so the sphere doesn't look stretched
    ax.set_box_aspect([1,1,1])
    
    # Optionally set axis limits based on table dimensions
    # ax.set_xlim([0, 1.5])
    # ax.set_ylim([-0.75, 0.75])
    # ax.set_zlim([0, 1.2])

    plt.legend()
    #plt.show()
    print("Saving plot to coverage.png...")
    plt.savefig('coverage.png', dpi=300, bbox_inches='tight')

if __name__ == "__main__":
    plot_coverage()