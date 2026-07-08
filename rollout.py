import os
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer

def setup_wsl_display():
    """Forces WSLg to use X11 to fix the GLFW window position bug."""
    os.environ['DISPLAY'] = ':0'
    if 'WAYLAND_DISPLAY' in os.environ:
        del os.environ['WAYLAND_DISPLAY']
    os.environ['MUJOCO_GL'] = 'glfw'

def create_scene():
    """Dynamically creates the scene XML and returns the model and data."""
    original_cwd = os.getcwd()
    panda_dir = "mujoco_menagerie/franka_emika_panda"
    
    # Ensure the directory exists to avoid errors
    if not os.path.exists(panda_dir):
        raise FileNotFoundError(f"Could not find {panda_dir}. Make sure you are in the right directory.")
        
    os.chdir(panda_dir)

    xml_string = """
    <mujoco>
      <include file="scene.xml"/>
      <worldbody>
        <body name="ball" pos="0.5 0 0.5">
          <freejoint name="ball_joint"/>
          <geom type="sphere" size="0.05" rgba="1 0 0 1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    temp_xml_path = "temp_scene.xml"
    with open(temp_xml_path, "w") as f:
        f.write(xml_string)

    model = mujoco.MjModel.from_xml_path(temp_xml_path)
    data = mujoco.MjData(model)

    os.remove(temp_xml_path)
    os.chdir(original_cwd)
    
    return model, data

def run_rollout(model, data, steps=500):
    """Runs the simulation loop with frame-skipping for smooth rendering."""
    # Pre-compute trajectory
    t = np.linspace(0, np.pi, steps)
    x_traj = 0.5 + 0.2 * np.cos(t)
    y_traj = 0.2 * np.sin(t)
    z_traj = 0.5 + 0.2 * np.sin(t)

    ball_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ball_joint")
    ball_qpos_adr = model.jnt_qposadr[ball_joint_id]

    # Target 60 FPS for the viewer to save CPU
    render_fps = 60
    render_interval = 1.0 / render_fps

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_render_time = time.time()
        
        for i in range(steps):
            # Update position
            data.qpos[ball_qpos_adr]     = x_traj[i]
            data.qpos[ball_qpos_adr + 1] = y_traj[i]
            data.qpos[ball_qpos_adr + 2] = z_traj[i]
            
            # Step physics
            mujoco.mj_step(model, data)
            
            # Frame skipping logic: Only sync viewer if enough time has passed
            current_time = time.time()
            if (current_time - last_render_time) >= render_interval:
                viewer.sync()
                last_render_time = current_time
            
            # Use a smaller sleep to prevent CPU locking, or rely on viewer.sync timing
            time.sleep(0.001) 
            
        print("Trajectory complete! You can now pan/rotate the camera.")
        
        # Keep alive loop
        while viewer.is_running():
            time.sleep(0.1)

if __name__ == "__main__":
    # Added argparse so you can easily change steps from the CLI!
    # Example: python rollout.py --steps 1000
    parser = argparse.ArgumentParser(description="Run a MuJoCo rollout with a Panda arm.")
    parser.add_argument("--steps", type=int, default=1000, help="Number of steps in the rollout.")
    args = parser.parse_args()

    setup_wsl_display()
    model, data = create_scene()
    run_rollout(model, data, steps=args.steps)