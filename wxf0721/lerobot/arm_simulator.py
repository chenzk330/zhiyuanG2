"""Arm simulator: TCP server on port 9002 that sends 6 joint angles oscillating 90<->80.

Protocol:
    Client sends `get\n`  -> server replies `[a1,...,a6]\n`
    Client sends `set [a1,...,a6]\n` -> server replies `ok\n` (target stored but sim keeps oscillating)

Angles change slowly from 90 to 80, back to 90, then to 80, repeatedly (sine wave, period ~8s).
"""

import json
import math
import socket
import threading
import time

HOST = "0.0.0.0"
PORT = 9002
PERIOD_S = 8.0  # seconds for a full 90->80->90 cycle


def current_angles(t: float) -> list[float]:
    """All 6 joints share the same oscillation; half-amplitude 5 around mean 85."""
    base = 85.0 + 5.0 * math.cos(2 * math.pi * t / PERIOD_S)
    return [round(base, 3)] * 6


def handle_client(conn: socket.socket, addr):
    print(f"[arm_sim] client connected from {addr}")
    start = time.monotonic()
    try:
        with conn:
            buf = bytearray()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buf.extend(data)
                while b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    request = line.decode("ascii").strip()
                    t = time.monotonic() - start
                    if request.startswith("set"):
                        # parse and store target (sim ignores it but acknowledges)
                        try:
                            payload = request[len("set") :].strip()
                            json.loads(payload)
                            reply = "ok\n"
                        except Exception:
                            reply = "error\n"
                        conn.sendall(reply.encode("ascii"))
                    elif request.startswith("get"):
                        angles = current_angles(t)
                        conn.sendall((json.dumps(angles) + "\n").encode("ascii"))
                    else:
                        # unknown request
                        conn.sendall(b"error\n")
    except ConnectionError:
        pass
    finally:
        print(f"[arm_sim] client disconnected: {addr}")


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(4)
        print(f"[arm_sim] listening on {HOST}:{PORT}  (6 joints, oscillating 90<->80, period {PERIOD_S}s)")
        try:
            while True:
                conn, addr = srv.accept()
                threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        except KeyboardInterrupt:
            print("\n[arm_sim] shutting down")


if __name__ == "__main__":
    main()
