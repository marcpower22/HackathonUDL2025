import cv2
import socket
import struct
import time

def main():
    # Configuration
    # REPLACE THIS WITH THE IP OF THE RECEIVER COMPUTER
    SERVER_IP = '172.17.197.9' 
    SERVER_PORT = 9999
    
    # Initialize Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set resolution to 640x360
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    client_socket = None
    prev_frame_time = 0
    target_fps = 10
    frame_interval = 1.0 / target_fps

    while True:
        try:
            if client_socket is None:
                print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.settimeout(5) # 5 second timeout for connection
                client_socket.connect((SERVER_IP, SERVER_PORT))
                client_socket.settimeout(None)
                print("Connected!")

            # Limit FPS
            current_time = time.time()
            if (current_time - prev_frame_time) < frame_interval:
                time.sleep(0.001) # Sleep briefly to avoid busy wait
                continue
            prev_frame_time = current_time

            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame (stream end?).")
                break

            # Resize frame to reduce bandwidth (optional)
            # frame = cv2.resize(frame, (640, 480))

            # Encode frame to JPEG
            result, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            
            if not result:
                continue

            data = encoded_frame.tobytes()
            
            # Pack the size of the data (unsigned long long, 8 bytes)
            message_size = struct.pack("Q", len(data)) 

            # Send size + data
            client_socket.sendall(message_size + data)

            # Optional: Display locally
            cv2.imshow('Sender (Local)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except (socket.error, socket.timeout) as e:
            print(f"Connection error: {e}")
            print("Retrying in 2 seconds...")
            if client_socket:
                client_socket.close()
            client_socket = None
            time.sleep(2)
        except KeyboardInterrupt:
            print("Stopping...")
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    if client_socket:
        client_socket.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
