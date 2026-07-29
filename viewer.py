import os
import time
import struct
import zmq
import numpy as np
import mujoco
import mujoco.viewer


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
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # subscribe to all topics

    nq = model.nq
    msg_size = nq * 8  # double = 8 bytes

    render_fps = 60
    render_interval = 1.0 / render_fps
    last_render_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            try:
                # Use non-blocking receive with small timeout
                # to keep the viewer responsive
                if socket.poll(timeout=10):  # ms
                    msg = socket.recv()
                    if len(msg) == msg_size:
                        qpos = np.frombuffer(msg, dtype=np.float64)
                        data.qpos[:] = qpos
                    else:
                        # size mismatch; ignore
                        pass
            except zmq.Again:
                pass

            # Step physics locally (optional; can be zero or tiny dt)
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