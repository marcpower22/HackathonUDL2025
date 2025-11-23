import cv2
import socket
import struct
import numpy as np
import threading
import time
import csv
import asyncio
import websockets
from collections import deque

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not found. Please install it: pip install ultralytics")
    exit(1)

try:
    from pyngrok import ngrok
except ImportError:
    ngrok = None

# Global variables
latest_frames = {} # {ip: (frame, timestamp)}
lock = threading.Lock()
running = True
unity_clients = [] # List of connected Unity clients (TCP sockets)
websocket_clients = set() # List of connected Unity clients (WebSockets)
websocket_loop = None # Reference to the asyncio loop for WebSockets

# --- CAMERA CONFIGURATION ---
# Assuming a room of roughly 4x4 meters.
# Origin (0,0,0) is the center of the room (the circle).
# Cam 1: Top-Left Corner (-200, -200, 200), looking at (0,0,0)
# Cam 2: Top-Right Corner (200, -200, 200), looking at (0,0,0)
# Units: Centimeters

def get_projection_matrix(cam_pos, target_pos):
    # Intrinsic Matrix (K) - Approximate for 640x360
    f = 400 # Focal length
    cx, cy = 320, 180
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]])

    # Extrinsic Matrix [R|t]
    # Forward vector (Z)
    z_axis = target_pos - cam_pos
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    # Up vector (Y) - assume world up is (0, 1, 0)
    # But OpenCV camera Y is down, so we invert logic slightly or just use standard LookAt
    # Standard LookAt:
    up = np.array([0, 1, 0])
    x_axis = np.cross(up, z_axis) # Right
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis) # True Up relative to camera
    
    # Rotation Matrix
    R = np.vstack([x_axis, y_axis, z_axis])
    
    # Translation Vector t = -R * C
    t = -R @ cam_pos
    
    # Extrinsic [R|t]
    Rt = np.column_stack((R, t))
    
    return K @ Rt

# Define positions (X, Y, Z) in cm
# Y is UP in 3D world, but cameras are usually high up looking down
# Let's assume cameras are at height 200cm, looking at ground (0)
center = np.array([0, 0, 0], dtype=float)

# List of cameras: (IP_ADDRESS, X, Y, Z)
# Add as many cameras as you have.
CAMERAS = [
    ('127.0.0.1', -200, 200, -200),      # Camera 1 (Top-Left)
    ('192.168.1.101', 200, 200, -200),   # Camera 2 (Top-Right)
    # ('192.168.1.102', -200, 200, 200), # Camera 3 (Bottom-Left) example
]

CAM_CONFIG = {}
for ip, x, y, z in CAMERAS:
    pos = np.array([x, y, z], dtype=float)
    CAM_CONFIG[ip] = {
        'P': get_projection_matrix(pos, center)
    }

def broadcast_to_unity(message):
    global unity_clients, websocket_clients, websocket_loop
    
    # 1. Send to TCP Clients (Desktop)
    dead_clients = []
    for client in unity_clients:
        try:
            client.sendall((message + "\n").encode('utf-8'))
        except Exception:
            dead_clients.append(client)
    
    for dead in dead_clients:
        unity_clients.remove(dead)
        try: dead.close()
        except: pass

    # 2. Send to WebSocket Clients (WebGL)
    if websocket_loop and websocket_clients:
        async def send_ws():
            if not websocket_clients: return
            # Create a copy to avoid runtime error if set changes during iteration
            for ws in list(websocket_clients):
                try:
                    await ws.send(message)
                except websockets.exceptions.ConnectionClosed:
                    websocket_clients.discard(ws)
                except Exception as e:
                    print(f"WS Error: {e}")
        
        asyncio.run_coroutine_threadsafe(send_ws(), websocket_loop)

async def websocket_handler(websocket):
    print(f"[WebSocket] Client connected")
    websocket_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        websocket_clients.discard(websocket)
        print(f"[WebSocket] Client disconnected")

async def start_websocket_server():
    global websocket_loop
    websocket_loop = asyncio.get_running_loop()
    print("[WebSocket] Server starting on port 5001...")
    async with websockets.serve(websocket_handler, "0.0.0.0", 5001):
        await asyncio.Future() # Run forever

def run_asyncio_loop():
    asyncio.run(start_websocket_server())

def unity_server_thread(listen_ip, listen_port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((listen_ip, listen_port))
    server.listen(5)
    print(f"[Unity Server] Listening on {listen_ip}:{listen_port}")
    
    while running:
        try:
            server.settimeout(1.0)
            conn, addr = server.accept()
            print(f"[Unity Client Connected] {addr}")
            unity_clients.append(conn)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Unity Server Error] {e}")

def handle_client(conn, addr):
    global latest_frames
    print(f"[New Connection] {addr}")
    ip = addr[0]
    
    data = b""
    payload_size = struct.calcsize("Q")

    try:
        while running:
            while len(data) < payload_size:
                packet = conn.recv(4096)
                if not packet: break
                data += packet
            if not data: break
            
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            
            while len(data) < msg_size:
                data += conn.recv(4096)
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is not None:
                with lock:
                    latest_frames[ip] = (frame, time.time())
                    
    except Exception as e:
        print(f"[Error] {addr}: {e}")
    finally:
        conn.close()
        print(f"[Disconnected] {addr}")

def server_thread(listen_ip, listen_port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((listen_ip, listen_port))
    server.listen(5)
    print(f"[Server] Listening on {listen_ip}:{listen_port}")
    
    while running:
        try:
            server.settimeout(1.0)
            conn, addr = server.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr))
            t.daemon = True
            t.start()
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Server Error] {e}")

def triangulate_point(projections, matrices):
    # DLT algorithm
    if len(projections) < 2:
        return None 
        
    A = []
    for (u, v), P in zip(projections, matrices):
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
        
    A = np.array(A)
    # SVD
    U, S, Vt = np.linalg.svd(A)
    X = Vt[-1]
    X = X / X[3] # Normalize
    return X[:3]

def main():
    global running
    
    # Load YOLO
    print("Loading YOLOv8 model...")
    model = YOLO('yolov8n-pose.pt') 
    
    # Start Server
    t_server = threading.Thread(target=server_thread, args=('0.0.0.0', 9999))
    t_server.start()

    # Start Unity Server
    t_unity = threading.Thread(target=unity_server_thread, args=('0.0.0.0', 5000))
    t_unity.start()

    # Start Unity Server (WebSocket)
    t_ws = threading.Thread(target=run_asyncio_loop, daemon=True)
    t_ws.start()
    
    # Setup Ngrok
    if ngrok:
        print("Starting ngrok tunnels...")
        try:

            # Open a HTTP tunnel for Unity WebGL (5001) -> WebSockets
            # WebGL clients connect via wss://<ngrok-url>
            public_url_ws = ngrok.connect(5001, "http").public_url
            # Convert http:// to wss:// for clarity
            wss_url = public_url_ws.replace("http://", "wss://").replace("https://", "wss://")
            print(f" * [Ngrok] Unity WebGL:    {wss_url}")
            print(f"   (Use this URL in your Unity WebGL build)")

        except Exception as e:
            print(f" * [Ngrok] Error starting tunnels: {e}")
            print("   (You might need to run 'ngrok config add-authtoken <token>')")
    else:
        print(" * [Ngrok] Not installed. To allow internet connections: pip install pyngrok")
    
    # CSV Logging
    csv_filename = f"3d_skeleton_log_{int(time.time())}.csv"
    f = open(csv_filename, 'w', newline='')
    writer = csv.writer(f)
    header = ['Timestamp', 'BodyID']
    for i in range(17): # YOLO pose has 17 keypoints
        header.extend([f'KP{i}_X', f'KP{i}_Y', f'KP{i}_Z'])
    writer.writerow(header)
    
    print("Processing started. Press 'q' to quit.")
    
    try:
        while True:
            with lock:
                current_frames = latest_frames.copy()
            
            if not current_frames:
                time.sleep(0.1)
                continue
                
            # Process frames
            keypoints_2d = {} # {ip: keypoints_array}
            
            for ip, (frame, ts) in current_frames.items():
                # Run YOLO
                results = model(frame, verbose=False)
                
                # Visualize
                annotated_frame = results[0].plot()
                cv2.imshow(f"Cam {ip}", annotated_frame)
                
                # Extract Keypoints (taking the first person detected)
                if results[0].keypoints is not None and len(results[0].keypoints) > 0:
                    # shape: (num_people, 17, 3) -> (x, y, conf)
                    kp = results[0].keypoints.data[0].cpu().numpy() 
                    keypoints_2d[ip] = kp
            
            # Triangulate if we have matching cameras in config
            valid_ips = [ip for ip in keypoints_2d if ip in CAM_CONFIG]
            
            if len(valid_ips) < 2:
                if len(keypoints_2d) > 0:
                    print(f"Waiting for 2nd camera for 3D... (Have {len(valid_ips)} valid: {valid_ips})")
            
            if len(valid_ips) >= 2:
                body_3d = []
                for kp_idx in range(17):
                    projections = []
                    matrices = []
                    
                    for ip in valid_ips:
                        kp = keypoints_2d[ip][kp_idx]
                        if kp[2] > 0.5: # Confidence threshold
                            projections.append((kp[0], kp[1]))
                            matrices.append(CAM_CONFIG[ip]['P'])
                    
                    if len(projections) >= 2:
                        pt_3d = triangulate_point(projections, matrices)
                        body_3d.append(pt_3d)
                    else:
                        body_3d.append(None)
                
                # Log to CSV
                row = [time.time(), 0] # ID 0
                for pt in body_3d:
                    if pt is not None:
                        row.extend([pt[0], pt[1], pt[2]])
                    else:
                        row.extend(['', '', ''])
                writer.writerow(row)

                # Broadcast to Unity
                # Format: Timestamp,BodyID,X0,Y0,Z0,X1,Y1,Z1...
                msg_parts = [str(time.time()), "0"]
                for pt in body_3d:
                    if pt is not None:
                        msg_parts.extend([str(pt[0]), str(pt[1]), str(pt[2])])
                    else:
                        msg_parts.extend(["", "", ""])
                broadcast_to_unity(",".join(msg_parts))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
                break
                
    except KeyboardInterrupt:
        running = False
    finally:
        f.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
