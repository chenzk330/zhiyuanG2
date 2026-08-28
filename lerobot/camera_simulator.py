"""Camera simulator: TCP server that sends a JPEG image with a black dot on white.

The image is a white background with a single black dot at the centre.
Frame is JPEG-encoded and length-prefixed (4-byte big-endian).

Protocol:
    Client sends `get\n` -> server replies <4-byte length><JPEG bytes>

Usage:
    python camera_simulator.py [--port 9003] [--width 640] [--height 480]
"""

import argparse
import socket
import struct
import threading

import cv2
import numpy as np

HOST = "0.0.0.0"
DEFAULT_PORT = 9003
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480


def make_frame(width: int, height: int) -> bytes:
    """Return a JPEG-encoded white image with a black dot at the centre."""
    img = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.circle(img, (width // 2, height // 2), radius=4, color=(0, 0, 0), thickness=-1)
    ok, jpg = cv2.imencode(".jpg", img)
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return jpg.tobytes()


def handle_client(conn: socket.socket, addr, width: int, height: int):
    print(f"[cam_sim] client connected from {addr}")
    try:
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                if b"get" in data:
                    jpg = make_frame(width, height)
                    conn.sendall(struct.pack(">I", len(jpg)) + jpg)
    except ConnectionError:
        pass
    finally:
        print(f"[cam_sim] client disconnected: {addr}")


def main():
    parser = argparse.ArgumentParser(description="TCP camera simulator")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, args.port))
        srv.listen(4)
        print(f"[cam_sim] listening on {HOST}:{args.port}  ({args.width}x{args.height}, white bg + black dot)")
        try:
            while True:
                conn, addr = srv.accept()
                threading.Thread(
                    target=handle_client,
                    args=(conn, addr, args.width, args.height),
                    daemon=True,
                ).start()
        except KeyboardInterrupt:
            print("\n[cam_sim] shutting down")


if __name__ == "__main__":
    main()
