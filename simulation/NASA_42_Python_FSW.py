#!/usr/bin/env python3
# Minimal NASA 42 HIL bridge: receive telem, send torques, exit when sim ends.

import socket, threading, time, re, math, sys
from typing import Optional, List

# --- Socket configuration (must match your Inp_IPC.txt) ---
TELEM_HOST, TELEM_PORT = "127.0.0.1", 10000   # 42 TX SERVER (we connect to this)
CMD_HOST,   CMD_PORT   = "0.0.0.0",  10001    # our SERVER (42 connects here)

# --- Spacecraft constants (update to your rotor inertia) ---
WHEEL_JR = 4.2946e-6  # kg*m^2
DEG2RAD  = math.pi / 180.0

# --- Shared state (extend as needed) ---
state = {
    "qbn_truth": None,   # SC[0].qbn (truth quaternion)       [q0,q1,q2,q3]
    "wb_truth":  None,   # SC[0].wb  (truth rates, rad/s)     [wx,wy,wz]
    "st_q":      None,   # ST[0].q   (meas quaternion)        [q0,q1,q2,q3]
    "gyro":      None,   # Gyro measured rates (rad/s)
    "whl_H":     {},     # Whl[i].H momentum (N*m*s)
    "whl_w":     {},     # Whl[i] rotor speed (rad/s)
}

# --- Finish signal: set when 42 ends or sockets drop ---
sim_done = threading.Event()

# --- Regex helpers ---
RE_RHS     = re.compile(r"=\s*(.+)$")
RE_WHL_IDX = re.compile(r"Whl\[(\d+)\]")

def parse_floats(s: str) -> List[float]:
    return [float(x) for x in s.strip().split()]

# ---------- Telemetry client (connects to 42 server @ 10000) ----------
def telem_client():
    """Connect once when 42 comes up; when the stream ends, signal sim_done."""
    # Wait for 42 to start; retry until connected (but don't reconnect after EOF).
    while not sim_done.is_set():
        try:
            sock = socket.create_connection((TELEM_HOST, TELEM_PORT), timeout=3)
            break
        except Exception:
            time.sleep(0.5)
    else:
        return  # sim_done set while waiting

    try:
        with sock:
            f = sock.makefile("r", encoding="utf-8", newline="\n")
            print("[TELEM] Connected to 42.", flush=True)
            for raw in f:
                line = raw.strip()
                if line:
                    sock.sendall(b'Ack\0')
                    handle_telem_line(line)
            # EOF reached (42 closed its TX socket) → sim finished
            print("[TELEM] EOF from 42 (sim ended).", flush=True)
    except Exception as e:
        print(f"[TELEM] Connection error: {e}", flush=True)
    finally:
        sim_done.set()

def handle_telem_line(line: str):
    # Examples:
    # SC[0].qbn = q0 q1 q2 q3
    # SC[0].wb  = wx wy wz
    # ST[0].q   = q0 q1 q2 q3
    # Gyro[0].Y = gx gy gz      (naming may vary; we just parse RHS triplet)
    # Whl[0].H  = H
    m = RE_RHS.search(line)
    if not m:
        return
    rhs = m.group(1)

    if line.startswith("SC[") and ".qbn" in line:
        state["qbn_truth"] = parse_floats(rhs)

    elif line.startswith("SC[") and ".wb" in line:
        state["wb_truth"] = parse_floats(rhs)

    elif line.startswith("ST[") and ".q" in line:
        state["st_q"] = parse_floats(rhs)

    elif "Gyro[" in line:
        try:
            g_deg = parse_floats(rhs)
            state["gyro"] = [v * DEG2RAD for v in g_deg]
        except Exception:
            pass

    elif "Whl[" in line and ".H" in line:
        try:
            H = parse_floats(rhs)[0]
        except Exception:
            return
        idxm = RE_WHL_IDX.search(line)
        if idxm:
            i = int(idxm.group(1))
            state["whl_H"][i] = H
            state["whl_w"][i] = H / WHEEL_JR

# ---------- Command server (42 connects as client @ 10001) ----------
class CmdServer:
    def __init__(self, host: str, port: int):
        self.host, self.port = host, port
        self._conn: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def _run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((self.host, self.port))
                srv.listen(1)
                print(f"[CMD] Listening on {self.host}:{self.port}", flush=True)
                conn, addr = srv.accept()
                print(f"[CMD] 42 connected from {addr}", flush=True)
                with self._lock:
                    self._conn = conn
                # Keep connection open until 42 disconnects
                try:
                    while not sim_done.is_set():
                        # 42 usually doesn't send here; just detect drop
                        data = conn.recv(4)
                        if not data:
                            print("[CMD] 42 disconnected.", flush=True)
                            break
                except Exception as e:
                    print(f"[CMD] Connection error: {e}", flush=True)
        finally:
            with self._lock:
                try:
                    if self._conn:
                        self._conn.close()
                except Exception:
                    pass
                self._conn = None
            sim_done.set()  # if command link drops, end the run

    def set_torque(self, tx: float, ty: float, tz: float):
        line = f"AC[0].Tcmd = {tx:.9e} {ty:.9e} {tz:.9e}\r\n"  # NEW: CRLF
        payload = line.encode("ascii", errors="strict")         # NEW: ASCII
        with self._lock:
            if self._conn:
                try:
                    self._conn.sendall(payload)
                    # print("HELLO")
                except Exception as e:
                    print(f"[CMD] Send failed: {e}", flush=True)
                    sim_done.set()

cmd = CmdServer(CMD_HOST, CMD_PORT)

# ---------- Controller hook ----------
def compute_torque():
    """
    Replace with your control law.
    Available inputs:
      state['st_q']      : star tracker quaternion (list[4])
      state['gyro']      : measured body rates (rad/s)
      state['whl_w']     : dict wheel_index -> speed (rad/s)
      state['wb_truth']  : truth rates (rad/s)   [debug only]
      state['qbn_truth'] : truth quaternion      [debug only]
    """
    # Start with zero torque keep-alive. Uncomment below for tiny PD test.
    tx = ty = tz = 0.0

    # # Example: very small rate-damping PD (uses measured gyro)
    # if state["gyro"] is not None:
    #     Kd = 5e-3  # N*m per (rad/s) — tune carefully
    #     gx, gy, gz = state["gyro"]
    #     tx, ty, tz = (-Kd*gx, -Kd*gy, -Kd*gz)

    return tx, ty, tz

# ---------- Main loop ----------
def main():
    cmd.start()
    threading.Thread(target=telem_client, daemon=True).start()

    dt = 0.1  # control period (choose k * DTSIM)
    print("[MAIN] Control loop running. Exits when 42 ends or links drop.", flush=True)

    try:
        while not sim_done.is_set():
            tx, ty, tz = compute_torque()
            cmd.set_torque(tx, ty, tz)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping by user.", flush=True)
        sim_done.set()

    # Small grace period for threads to wind down
    time.sleep(0.2)
    print("[MAIN] Done.", flush=True)

if __name__ == "__main__":
    # In Spyder, use the red stop or Ctrl+C to end early;
    # otherwise this will exit automatically when 42 finishes.
    try:
        main()
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        sys.exit(1)
