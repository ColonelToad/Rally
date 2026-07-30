import os
import time
import struct
import zmq
import numpy as np
import mujoco
import mujoco.viewer

human_command = {
    "serv_ball": False,
    "delta_pos": np.zeros(3) # [dx, dy, dz] controlled by keyboard
}

def keyboard_callback(keycode):
    # Key codes in MuJoCo viewer (ASCII values)
    # Spacebar (32): Serve / Reset Ball
    if keycode == 32:
        human_command["serv_ball"] = True
        print("[Human Player] Triggered manual ball serve!")
    
    # W / S : Move paddle forward / backward (X-axis)
    elif keycode == ord('w'):
        human_command["delta_pos"][0] += 0.05
    elif keycode == ord('s'):
        human_command["delta_pos"][0] -= 0.05
        
    # A / D : Move paddle left / right (Y-axis)
    elif keycode == ord('a'):
        human_command["delta_pos"][1] += 0.05
    elif keycode == ord('d'):
        human_command["delta_pos"][1] -= 0.05


def setup_wsl_display():
    os.environ['DISPLAY'] = ':0'
    if 'WAYLAND_DISPLAY' in os.environ:
        del os.environ['WAYLAND_DISPLAY']
    os.environ['MUJOCO_GL'] = 'glfw'


def create_model():
    model = mujoco.MjModel.from_xml_path("panda_hit_scene.xml")
    data = mujoco.MjData(model)
    return model, data


def run_viewer(model, data):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5556")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    nq = model.nq
    msg_size = nq * 8

    render_fps = 60
    render_interval = 1.0 / render_fps
    last_render_time = time.time()

    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard_callback) as viewer:
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -20.0
        viewer.cam.distance = 4.5
        viewer.cam.lookat[0] = 0.0
        viewer.cam.lookat[1] = 0.0
        viewer.cam.lookat[2] = 0.76
        viewer.sync()

        while viewer.is_running():
            try:
                if socket.poll(timeout=10):
                    msg = socket.recv()
                    if len(msg) == msg_size:
                        qpos = np.frombuffer(msg, dtype=np.float64)
                        data.qpos[:] = qpos
            except zmq.Again:
                pass

            # Handle Human Commands locally or send to C++ backend
            if human_command["serv_ball"]:
                # Find ball joint qpos address and launch it towards Arm A
                ball_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ball_joint")
                if ball_joint_id >= 0:
                    qpos_adr = model.jnt_qposadr[ball_joint_id]
                    dof_adr = model.jnt_dofadr[ball_joint_id]
                    
                    # Reset ball position to center of table above net, moving towards Arm A
                    data.qpos[qpos_adr : qpos_adr+3] = [1.2, 0.0, 1.2] # Start on right side
                    data.qvel[dof_adr : dof_adr+3] = [-3.0, 0.0, 1.0] # Velocity towards left
                human_command["serv_ball"] = False

            mujoco.mj_forward(model, data)

            current_time = time.time()
            if (current_time - last_render_time) >= render_interval:
                viewer.sync()
                last_render_time = current_time

            time.sleep(0.001)

    socket.close()
    context.term()


if __name__ == "__main__":
    setup_wsl_display()
    model, data = create_model()
    run_viewer(model, data)