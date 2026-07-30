import os
import time
import zmq
import numpy as np
import mujoco
import mujoco.viewer
from brain import TacticalCoach

human_command = {
    "serv_ball": False,
    "run_stress_test": False,
    "delta_pos": np.zeros(3) 
}

def keyboard_callback(keycode):
    try:
        # Safely convert GLFW keycode to lowercase character
        key = chr(keycode).lower()
    except (ValueError, OverflowError):
        return

    # Spacebar (Keycode 32)
    if key == ' ':
        human_command["serv_ball"] = True
        print("[Human Player] Triggered manual ball serve!")
        if human_command["serv_ball"]:
            try:
                cmd_socket.send_string("SERVE", flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            human_command["serv_ball"] = False
            
            # Construct a post-rally breakdown of the point that just ended
            mock_post_rally_summary = {
                "rally_length": 6,
                "winner": "Arm_A",
                "loser": "Arm_B",
                "miss_reason": "Arm_B failed left-side edge return",
                "last_ball_y": 0.38
            }
            
            # Trigger LLM in background while the serve executes
            coach.evaluate_rally_async(mock_post_rally_summary)
        
    # 't' Key: Run LLM Benchmark
    elif key == 't':
        human_command["run_stress_test"] = True
        print("\n--- STARTING LIVE LLM STRESS TEST IN VIEWER ---")
        
    # Movement Keys (W, A, S, D)
    elif key == 'w':
        human_command["delta_pos"][0] += 0.05
    elif key == 's':
        human_command["delta_pos"][0] -= 0.05
    elif key == 'a':
        human_command["delta_pos"][1] += 0.05
    elif key == 'd':
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

def execute_stress_test(coach):
    sample_match_states = [
        "Volley 1: Ball returned successfully to center. Opponent reaction time normal.",
        "Volley 2: Ball hit net edge. Opponent scrambled left to retrieve.",
        "Volley 3: High-speed cross-court smash executed. Opponent missed right.",
        "Volley 4: Rally extended to 14 shots. Opponent playing deep defensive lobs.",
        "Volley 5: Unforced error on left flank. Immediate tactical adjustment required."
    ]
    
    print("[Stress Test] Saturating coach queue with 5 rapid game states...")
    for state in sample_match_states:
        start_time = time.perf_counter()
        coach.evaluate_async(state)
        dispatch_delay = time.perf_counter() - start_time
        print(f"[Stress Test] Dispatched state in {dispatch_delay*1000:.3f} ms")
        time.sleep(0.05) # Brief sleep between rapid inputs

def run_viewer(model, data, coach):
    context = zmq.Context()
    
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5556")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    cmd_socket = context.socket(zmq.PUSH)
    cmd_socket.setsockopt(zmq.LINGER, 0)
    cmd_socket.connect("tcp://localhost:5557")

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
                if socket.poll(timeout=1):
                    msg = socket.recv()
                    if len(msg) == msg_size:
                        qpos = np.frombuffer(msg, dtype=np.float64)
                        data.qpos[:] = qpos
            except zmq.Again:
                pass

            if human_command["serv_ball"]:
                try:
                    cmd_socket.send_string("SERVE", flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
                human_command["serv_ball"] = False
                
            if human_command["run_stress_test"]:
                execute_stress_test(coach)
                human_command["run_stress_test"] = False

            mujoco.mj_forward(model, data)

            current_time = time.time()
            if (current_time - last_render_time) >= render_interval:
                viewer.sync()
                last_render_time = current_time

            time.sleep(0.001)

    socket.close()
    cmd_socket.close()
    context.term()

if __name__ == "__main__":
    setup_wsl_display()
    
    # Load model upfront before launching MuJoCo window
    coach = TacticalCoach("models/qwen2.5-1.5b-instruct-q2_k.gguf", physical_cores=2)
    
    model, data = create_model()
    run_viewer(model, data, coach)