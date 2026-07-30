import threading
import time
import json
import re
import zmq
from llama_cpp import Llama

class TacticalCoach:
    def __init__(self, model_path, physical_cores=2):
        print("[Coach] Loading Post-Rally Strategist model...")
        self.llm = Llama(
            model_path=model_path, 
            n_gpu_layers=0,        
            n_ctx=256,             
            n_threads=physical_cores, 
            verbose=False
        )
        
        context = zmq.Context()
        self.socket = context.socket(zmq.PUSH)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind("tcp://*:5558")
        
        self.latest_rally_summary = None
        self.state_lock = threading.Lock()
        
        self.is_thinking = False
        self.current_strategy = "RALLY"
        
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def evaluate_rally_async(self, rally_summary_dict):
        """Pass completed rally stats to the coach."""
        with self.state_lock:
            self.latest_rally_summary = rally_summary_dict
            print(f"[Coach] Received post-rally summary for analysis: {rally_summary_dict.get('winner', 'Unknown')} won.")

    def _worker_loop(self):
        print("[Worker] Post-rally standby thread ready.")
        while True:
            summary_to_process = None
            
            while summary_to_process is None:
                with self.state_lock:
                    summary_to_process = self.latest_rally_summary
                    self.latest_rally_summary = None 
                if summary_to_process is None:
                    time.sleep(0.05)
            
            self.is_thinking = True
            
            # Post-Rally Prompting
            prompt = f"""[INST] You are a table tennis coach reviewing the last completed rally.
Rally Summary: {json.dumps(summary_to_process)}

Decide tactical adjustments for the NEXT point.
Options: RALLY (neutral), AIM_LEFT (exploit left flank), AIM_RIGHT (exploit right flank), DEFENSIVE (absorb speed).

Output RAW JSON ONLY using this structure:
{{"strategy": "MODE", "target_offset_y": 0.0, "aggression_factor": 1.0}}
[/INST]"""

            try:
                start_time = time.time()
                response = self.llm(
                    prompt, 
                    max_tokens=64,
                    temperature=0.2, 
                    stop=["\n\n", "[/INST]"]
                )
                raw_text = response["choices"][0]["text"].strip()
                
                json_match = re.search(r'\{.*?\}', raw_text, re.DOTALL)
                if json_match:
                    clean_json = json_match.group(0)
                    data = json.loads(clean_json)
                    latency = time.time() - start_time
                    
                    self.current_strategy = data.get("strategy", "RALLY")
                    print(f"\n[LLM Coach Post-Rally Analysis] -> {data} (Computed in {latency:.2f}s)")
                    
                    # Push tactical mode to C++ for the next rally
                    try:
                        self.socket.send_string(json.dumps(data), flags=zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                else:
                    print(f"[LLM Coach Error] No valid JSON found: '{raw_text}'")
                    
            except Exception as e:
                print(f"[LLM Coach Error] Parsing failed: {e}")
                
            finally:
                self.is_thinking = False