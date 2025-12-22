import time
import socket
import struct
import threading
import numpy as np
import mujoco
import mujoco.viewer

# --- CONFIGURATION ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Shared state between UDP thread and Simulation
# [x, y, z, qw, qx, qy, qz]
# Default to z=0.5 so it doesn't spawn inside the floor
current_pose = [0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
lock = threading.Lock()
running = True

# --- MJCF MODEL (XML) ---
# A simple scene with a checkered floor and a "floating" object 
# representing your end effector.
xml_string = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="308"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 .05" type="plane" material="grid" condim="3"/>

    <body name="end_effector" pos="0 0 0.5">
      <freejoint name="ee_joint"/>
      
      <geom type="box" size=".05 .05 .05" rgba="0.9 0.2 0.2 1" />
      
      <geom fromto="0 0 0 0.1 0 0" size="0.005" rgba="1 0 0 1" type="cylinder"/>
      <geom fromto="0 0 0 0 0.1 0" size="0.005" rgba="0 1 0 1" type="cylinder"/>
      <geom fromto="0 0 0 0 0 0.1" size="0.005" rgba="0 0 1 1" type="cylinder"/>
    </body>
  </worldbody>
</mujoco>
"""

def udp_listener():
    """Background thread to receive UDP packets continuously."""
    global current_pose, running
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.2) # Allow loop to check 'running' flag

    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}...")

    while running:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == 28:
                # Unpack Little Endian (<) 7 floats: x, y, z, qw, qx, qy, qz
                values = struct.unpack('<fffffff', data)
                
                with lock:
                    current_pose = list(values)
                    
        except socket.timeout:
            continue
        except Exception as e:
            print(f"UDP Error: {e}")

    sock.close()
    print("[UDP] Listener stopped.")

def main():
    global running
    
    # 1. Load the Model
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # 2. Start UDP Thread
    t = threading.Thread(target=udp_listener)
    t.start()

    print("[SIM] Starting MuJoCo Viewer. Close window to exit.")

    # 3. Launch Viewer
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                # --- SYNC UDP DATA TO SIMULATION ---
                with lock:
                    # MuJoCo freejoint qpos structure: [x, y, z, w, x, y, z]
                    # Your UDP data matches this order: pos(3) + quat(4)
                    
                    # We assign the received values directly to the joint position
                    data.qpos[0:7] = current_pose

                # Step the simulation (calculates velocities/accelerations, though we are teleporting)
                mujoco.mj_forward(model, data)
                
                # Sync viewer
                viewer.sync()
                
                # Sleep to match ~60Hz or simulation timestep
                time.sleep(0.016)
                
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        t.join()
        print("[SIM] Closed.")

if __name__ == "__main__":
    main()