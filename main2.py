import socket

HOST = ""
PORT = 5001
OUTPUT_FILE = "output.png"

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)

        print(f"[+] Server listening on port {PORT}...")

        while True:
            conn, addr = s.accept()
            print(f"[+] Connected by {addr}")

            data = b""
            while True:
                packet = conn.recv(4096)
                if not packet:
                    break
                data += packet

            text = data.decode("utf-8")

            # ----- SAVE TO FILE -----
            with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
                f.write(text)

            print("\n===== Received File Update =====\n")
            print(text)
            print("\n[+] File saved to:", OUTPUT_FILE)
            print("========= End of Update ==========\n")

if __name__ == "__main__":
    start_server()






"""import socket

HOST = ""
PORT = 5001

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)

        print(f"[+] Server listening on port {PORT}...")

        while True:
            conn, addr = s.accept()
            print(f"[+] Connected by {addr}")

            data = b""
            while True:
                packet = conn.recv(4096)
                if not packet:
                    break
                data += packet

            print("\n===== Received File Update =====\n")
            print(data.decode("utf-8"))
            print("\n========= End of Update ==========\n")

if __name__ == "__main__":
    start_server()


"""
