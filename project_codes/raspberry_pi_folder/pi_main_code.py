import cv2
import numpy as np
import struct
import serial
import time
import socket
import threading
import heapq
import os
import queue
from picamera2 import Picamera2
import dashboard

# ==========================================================================
# GRAPH-BASED NAVIGATION  (replaces ROUTE_VARIANTS / REROUTE_TAILS)
# --------------------------------------------------------------------------
# The car no longer uses fixed action lists. It localizes on a graph map:
#   - you give it a start node + heading + destination at startup
#   - Dijkstra computes the shortest path that avoids blocked edges
#   - each graph step is converted to a relative action based on heading
#   - YOLO / keyboard closed-road signs block edges and trigger replanning
#
# The low-level motion is unchanged: it still follows colored lines with
# OpenCV/PID, detects junctions, and sends vx/vy/wz/target_yaw/mode to STM32.
#
# Junction camera types (used ONLY to know "a node was reached"):
#   "BOTH"       -> no_line  (both branches open)
#   "LEFT_ONLY"  -> only_right
#   "RIGHT_ONLY" -> only_left
#
# Actions: "STRAIGHT", "LEFT", "RIGHT", "UTURN", "STOP"
#
# Keyboard one-shot rules (applied at the next junction):
#   l = LEFT forbidden    t = RIGHT forbidden
#   f = FORWARD forbidden n = clear all rules
# YOLO auto rules from PC:
#   SAGKAPALI -> RIGHT forbidden     SOLKAPALI -> LEFT forbidden
#   ILERIKAPALI -> FORWARD forbidden (girisyok / no-entry; reroutes, U-turns if needed)
# ==========================================================================

DIRECTIONS = ["NORTH", "EAST", "SOUTH", "WEST"]

GRAPH = {
    "A": {
        "SOUTH": {"to": "A_EXIT", "cost": 1},
    },
    "A_EXIT": {
        "NORTH": {"to": "A", "cost": 1},
        "WEST":  {"to": "CROSS_CENTER", "cost": 4},
        "SOUTH": {"to": "LOWER_RIGHT", "cost": 2},
    },
    "C": {
        "SOUTH": {"to": "CROSS_CENTER", "cost": 1},
    },
    "CROSS_CENTER": {
        "NORTH": {"to": "C", "cost": 1},
        "EAST":  {"to": "A_EXIT", "cost": 4},
        "WEST":  {"to": "LEFT_ROAD", "cost": 4},
        "SOUTH": {"to": "LOWER_CENTER", "cost": 2},
    },
    "LEFT_ROAD": {
        "EAST":  {"to": "CROSS_CENTER", "cost": 4},
        "SOUTH": {"to": "B_EXIT", "cost": 2},
    },
    "B": {
        "NORTH": {"to": "B_EXIT", "cost": 1},
    },
    "B_EXIT": {
        "NORTH": {"to": "LEFT_ROAD", "cost": 2},
        "EAST":  {"to": "LOWER_CENTER", "cost": 4},
        "SOUTH": {"to": "B", "cost": 1},
    },
    "LOWER_CENTER": {
        "NORTH": {"to": "CROSS_CENTER", "cost": 2},
        "WEST":  {"to": "B_EXIT", "cost": 4},
        "EAST":  {"to": "LOWER_RIGHT", "cost": 4},
    },
    "LOWER_RIGHT": {
        "NORTH": {"to": "A_EXIT", "cost": 2},
        "WEST":  {"to": "LOWER_CENTER", "cost": 4},
    },
}

# ----- live navigation state (replaces the old route_* globals) -----
current_node = None            # node the car is currently at / has just reached
current_heading = None         # direction the car is travelling on the current edge
destination = None             # target node
planned_path = None            # list of nodes from current_node to destination
blocked_edges = set()          # set of (node, direction) that must not be used
pending_next_node = None       # node the in-progress movement leads to
pending_target_direction = None  # heading the car will have once the movement finishes
previous_node = None           # node we came FROM (used for NO-PATH U-turn recovery)
no_path = False                # True when Dijkstra cannot reach the destination
forbidden_rules = set()        # active one-shot rules: "LEFT" / "RIGHT" / "FORWARD"
forced_turn_rule = None       # active one-shot YOLO turn rule: "LEFT" / "RIGHT"

# remembered startup values so a reset can restart without re-typing them
init_node = None
init_heading = None
init_dest = None

PORT = "/dev/serial0"
BAUD = 115200
CMD_FMT = "<HhhhfBB"
CMD_HDR = 0xAA55
MODE_ROTATE_YAW = 1
MODE_DRIVE_HEADING = 2

FORWARD_SPEED = 3000
WZ_MAX = 2500

CAM_W = 640
CAM_H = 480
ROI_TOP = 0
ROI_BOTTOM = CAM_H

BANDS_Y = [int(CAM_H * 0.45),   # dotY1
           int(CAM_H * 0.60),   # dotY2 -> ONE LINE / TURN
           int(CAM_H * 0.75)]   # dotX  -> TWO LINE / FOLLOW
BAND_HALF = 12
ROAD_HALF_WIDTH = 160
ONE_LINE_SCAN_HALF_H = 60

MIN_ONE_LINE_AREA = 900
MIN_ONE_LINE_LEN = 60

lower_red1 = np.array([0, 88, 10]);    upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 88, 10]);  upper_red2 = np.array([180, 255, 255])
lower_pink = np.array([140, 40, 100]); upper_pink = np.array([170, 255, 255])
lower_blue = np.array([100, 50, 50]);  upper_blue = np.array([130, 255, 255])
lower_green = np.array([78, 180, 0]);  upper_green = np.array([94, 255, 255])

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

KP = 4.0
KI = 0.0
KD = 0.7
pid_integral = 0.0
pid_last_err = 0.0
pid_last_time = time.time()

KP_X = 8.0
CENTER_TOL = 10
MIN_STRAFE = 3500
MAX_STRAFE = 5500
ERROR_ALPHA = 0.65
smooth_error = 0.0
last_road_center = CAM_W // 2

STATE_FOLLOW = 0
STATE_FORWARD_GAP = 1
STATE_TURN = 2
STATE_AFTER_TURN_SIDE = 3
STATE_STRAIGHT_PASS = 4
STATE_DONE = 5
STATE_NO_PATH = 6          # NEW: no route to destination -> stop
STATE_UTURN_BACK = 7       # NEW: after a 180 U-turn, drive back along the road briefly

state = STATE_FOLLOW
state_start = time.time()

ONE_LINE_LIMIT = 40
one_line_counter = 0
NO_LINE_LIMIT = 30
no_line_counter = 0
STOP_CONFIRM_LIMIT = 12

FORWARD_GAP_TIME = 1.85
ONE_LINE_GAP_TIME = 1.60
forward_gap_time = FORWARD_GAP_TIME
STRAIGHT_PASS_TIME = 1.80

TURN_TIME = 2.5
UTURN_TIME = 4.0           # 180 deg needs longer; tune on the car
UTURN_BACK_TIME = 1.5      # after the 180, drive backward this long; tune on the car
UTURN_BACK_SPEED = 2500    # backward speed after the U-turn (lower than FORWARD_SPEED)
RIGHT_TURN_ANGLE = -90.0
LEFT_TURN_ANGLE = 90.0
UTURN_ANGLE = 180.0
pending_turn_angle = RIGHT_TURN_ANGLE
pending_uturn_retreat_after_turn = False  # True only for UTURN: turn 180 first, then go back
target_yaw = 0.0

# ==========================================================================
# YOLO STREAMING (to PC)  --  navigation is NOT affected by this.
# ==========================================================================
PC_HOST = "10.75.215.174"
PC_PORT = 5555
YOLO_SEND_W = 320
YOLO_SEND_H = 240
YOLO_JPEG_QUALITY = 60
YOLO_NONE_LABEL = "YOK"
YOLO_LOG_PATH = "/home/ehsan/carcodes/yolo_detections.log"

yolo_label = "YOK"
yolo_conf = 0.0
yolo_lock = threading.Lock()
_last_logged_label = None

latest_frame = None
latest_frame_lock = threading.Lock()

yolo_running = True

YOLO_FORBIDDEN_CONF = 0.50
yolo_last_forbidden_cmd = None
yolo_last_forbidden_time = 0.0
yolo_forbidden_cooldown = 1.0
yolo_rule_status = "NONE"
yolo_last_forced_cmd = None
yolo_last_forced_time = 0.0
yolo_forced_cooldown = 1.0


def send_frame(sock, frame):
    small = cv2.resize(frame, (YOLO_SEND_W, YOLO_SEND_H))
    ok, buf = cv2.imencode(".jpg", small,
                           [cv2.IMWRITE_JPEG_QUALITY, YOLO_JPEG_QUALITY])
    if not ok:
        return
    data = buf.tobytes()
    sock.sendall(struct.pack(">I", len(data)) + data)


def recv_result(sock):
    data = b""
    while b"\n" not in data:
        chunk = sock.recv(64)
        if not chunk:
            return None
        data += chunk
    return data.decode(errors="ignore").strip()


def _yolo_stream_loop(sock):
    """Ship frames while connected and read 'label:conf' results back."""
    global yolo_label, yolo_conf, _last_logged_label
    while yolo_running:
        with latest_frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
        if frame is None:
            time.sleep(0.05)
            continue

        send_frame(sock, frame)
        result = recv_result(sock)
        if not result:
            return

        parts = result.split(":")
        label = parts[0].strip()
        conf = 0.0
        if len(parts) > 1:
            try:
                conf = float(parts[1])
            except ValueError:
                conf = 0.0

        with yolo_lock:
            yolo_label = label
            yolo_conf = conf

        if label and label != YOLO_NONE_LABEL and label != _last_logged_label:
            _last_logged_label = label
            stamp = time.strftime("%H:%M:%S")
            line = f"[{stamp}] DETECTED: {label} ({conf:.2f})"
            print(line)
            try:
                with open(YOLO_LOG_PATH, "a") as f:
                    f.write(line + "\n")
            except Exception:
                pass
        elif label == YOLO_NONE_LABEL:
            _last_logged_label = None


def yolo_thread_fn():
    """Connect (with retry) and stream frames without ever blocking the main loop."""
    announced = False
    while yolo_running:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((PC_HOST, PC_PORT))
            sock.settimeout(None)
            print(f"[YOLO] connected to PC {PC_HOST}:{PC_PORT}")
            announced = True
            _yolo_stream_loop(sock)
        except Exception as e:
            if announced or not yolo_running:
                print(f"[YOLO] connection lost: {e}")
            announced = False
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        if not yolo_running:
            break
        for _ in range(20):
            if not yolo_running:
                break
            time.sleep(0.1)


def calc_checksum(data):
    return sum(data[:-1]) & 0xFF


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle(a):
    return (a + 180.0) % 360.0 - 180.0


def send_command(ser, vx=0, vy=0, wz=0, target_yaw=0.0, mode=MODE_DRIVE_HEADING):
    raw = struct.pack(CMD_FMT, CMD_HDR,
                      int(vx), int(vy), int(wz),
                      float(target_yaw), int(mode), 0)
    frame = raw[:-1] + bytes([calc_checksum(raw)])
    ser.write(frame)


def reset_pid():
    global pid_integral, pid_last_err, pid_last_time
    pid_integral = 0.0
    pid_last_err = 0.0
    pid_last_time = time.time()


def reset_lane_memory():
    global smooth_error, last_road_center
    smooth_error = 0.0
    last_road_center = CAM_W // 2


def pid_update(error):
    global pid_integral, pid_last_err, pid_last_time
    now = time.time()
    dt = max(now - pid_last_time, 0.001)
    pid_last_time = now
    pid_integral = clamp(pid_integral + error * dt, -500, 500)
    deriv = (error - pid_last_err) / dt
    pid_last_err = error
    output = KP * error + KI * pid_integral + KD * deriv
    return int(clamp(output, -WZ_MAX, WZ_MAX))


# ======================================================================
# GRAPH PLANNER
# ======================================================================
def dijkstra(graph, start, dest, blocked):
    """Shortest path (list of nodes) from start to dest, skipping blocked edges.
    Returns None if no path exists."""
    if start == dest:
        return [start]
    dist = {start: 0}
    prev = {}
    pq = [(0, start)]
    visited = set()
    while pq:
        d, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        if node == dest:
            break
        for direction, edge in graph.get(node, {}).items():
            if (node, direction) in blocked:
                continue
            nxt = edge["to"]
            nd = d + edge.get("cost", 1)
            if nd < dist.get(nxt, float("inf")):
                dist[nxt] = nd
                prev[nxt] = node
                heapq.heappush(pq, (nd, nxt))
    if dest not in dist:
        return None
    path = [dest]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def get_direction_between(node1, node2):
    """Absolute direction (NORTH/EAST/SOUTH/WEST) of the edge node1 -> node2."""
    for direction, edge in GRAPH.get(node1, {}).items():
        if edge["to"] == node2:
            return direction
    return None


def direction_to_action(heading, target_direction):
    """Convert an absolute target direction into a relative car action."""
    if target_direction is None:
        return "STOP"
    i = DIRECTIONS.index(heading)
    j = DIRECTIONS.index(target_direction)
    diff = (j - i) % 4
    if diff == 0:
        return "STRAIGHT"
    elif diff == 1:
        return "RIGHT"
    elif diff == 3:
        return "LEFT"
    else:
        return "UTURN"


def plan_route():
    """(Re)compute planned_path from current_node to destination."""
    global planned_path, no_path
    path = dijkstra(GRAPH, current_node, destination, blocked_edges)
    if path is None:
        planned_path = None
        no_path = True
        print(f"[PLAN] NO PATH {current_node} -> {destination} (blocked={sorted(blocked_edges)})")
        return False
    planned_path = path
    no_path = False
    print(f"[PLAN] {current_node} -> {destination}: {path}")
    return True


def replan_after_block():
    """Re-run Dijkstra after an edge was blocked."""
    return plan_route()


def get_next_planned_step():
    """Info about the next junction the car will act on (the node planned_path[1]).
    next_action is what it will DO there (STRAIGHT/LEFT/RIGHT/UTURN/STOP)."""
    if no_path or planned_path is None:
        return {"next_node": None, "target_direction": None, "next_action": "NO_PATH"}
    if current_node == destination or len(planned_path) < 2:
        return {"next_node": None, "target_direction": None, "next_action": "STOP"}
    upcoming = planned_path[1]
    if upcoming == destination:
        return {"next_node": upcoming, "target_direction": None, "next_action": "STOP"}
    if len(planned_path) < 3:
        return {"next_node": upcoming, "target_direction": None, "next_action": "STOP"}
    after = planned_path[2]
    arrival_heading = get_direction_between(planned_path[0], upcoming) or current_heading
    tgt = get_direction_between(upcoming, after)
    return {"next_node": upcoming, "target_direction": tgt,
            "next_action": direction_to_action(arrival_heading, tgt)}


def next_action():
    return get_next_planned_step()["next_action"]


def block_current_planned_edge(reason=""):
    """Block the immediate edge current_node -> planned_path[1]."""
    if planned_path is None or len(planned_path) < 2:
        return False
    d = get_direction_between(current_node, planned_path[1])
    if d is None:
        return False
    edge = (current_node, d)
    if edge in blocked_edges:
        return False
    blocked_edges.add(edge)
    print(f"[BLOCK] planned edge {edge} blocked ({reason})")
    return True


def block_forbidden_edges(node, heading):
    """Turn the active relative forbidden_rules into absolute blocked edges
    at `node` (handles combos like LEFT+FORWARD). Returns True if anything new
    was blocked."""
    if heading not in DIRECTIONS:
        return False
    i = DIRECTIONS.index(heading)
    changed = False
    for rule in forbidden_rules:
        if rule == "FORWARD":
            d = DIRECTIONS[i]
        elif rule == "RIGHT":
            d = DIRECTIONS[(i + 1) % 4]
        elif rule == "LEFT":
            d = DIRECTIONS[(i + 3) % 4]
        else:
            continue
        edge = (node, d)
        if edge not in blocked_edges:
            blocked_edges.add(edge)
            changed = True
            print(f"[BLOCK] {rule} (heading {heading}) -> edge {edge} blocked")
    return changed


def relative_turn_to_direction(heading, rule):
    """Convert a relative one-shot turn rule (LEFT/RIGHT/FORWARD) into an
    absolute graph direction based on the car heading at the junction."""
    if heading not in DIRECTIONS:
        return None
    i = DIRECTIONS.index(heading)
    if rule == "FORWARD":
        return DIRECTIONS[i]
    if rule == "RIGHT":
        return DIRECTIONS[(i + 1) % 4]
    if rule == "LEFT":
        return DIRECTIONS[(i + 3) % 4]
    return None


def apply_forced_turn_rule(now):
    """Force the next junction decision from a YOLO SAG/SOL sign.

    Example: if SAG is detected before the junction, the car must choose the
    RIGHT edge at that junction if that edge exists and is not blocked. The
    edge that the planner originally wanted to use is then blocked, because
    the sign proved that this planned choice is not allowed here. After choosing
    the forced edge, Dijkstra is run from the node after the forced turn to
    build a fresh route to the destination.

    Returns a text message if a forced rule was consumed, otherwise None.
    """
    global planned_path, no_path, forced_turn_rule

    if forced_turn_rule is None:
        return None

    rule = forced_turn_rule
    forced_turn_rule = None

    forced_direction = relative_turn_to_direction(current_heading, rule)
    if forced_direction is None:
        return f"[FORCE] {rule} ignored: invalid heading {current_heading}"

    edge = GRAPH.get(current_node, {}).get(forced_direction)
    if edge is None:
        return f"[FORCE] {rule} ignored @{current_node}: no {forced_direction} road"

    if (current_node, forced_direction) in blocked_edges:
        return f"[FORCE] {rule} ignored @{current_node}: {forced_direction} is already blocked"

    # The current planned edge is what the algorithm wanted before YOLO forced
    # another direction. Block it so Dijkstra will not select the same wrong
    # way again after the forced turn.
    original_direction = None
    original_next = None
    blocked_original_msg = ""
    if planned_path is not None and len(planned_path) >= 2:
        original_next = planned_path[1]
        original_direction = get_direction_between(current_node, original_next)

    if (original_direction is not None
            and original_direction != forced_direction
            and (current_node, original_direction) not in blocked_edges):
        blocked_edges.add((current_node, original_direction))
        blocked_original_msg = f"; blocked old planned edge {current_node}:{original_direction}->{original_next}"
        print(f"[BLOCK] old planned edge {(current_node, original_direction)} blocked "
              f"because YOLO forced {rule}")

    forced_next = edge["to"]
    tail = dijkstra(GRAPH, forced_next, destination, blocked_edges)
    if tail is None:
        # The mandatory turn exists, but there is no route from that branch to
        # the destination with the current blocked edges. Let the normal no-path
        # recovery handle it after this function returns.
        planned_path = [current_node, forced_next]
        no_path = True
        return (f"[FORCE] {rule} @{current_node}: forced {forced_direction} -> {forced_next}"
                f"{blocked_original_msg}; no route from {forced_next} to {destination}")

    planned_path = [current_node] + tail
    no_path = False
    print(f"[FORCE] YOLO {rule} @{current_node}: taking {forced_direction} -> {forced_next}"
          f"{blocked_original_msg}; new plan {planned_path}")
    return (f"[FORCE] {rule} -> {forced_direction} via {forced_next}"
            f"{blocked_original_msg}; replanned {planned_path}")


def update_position_after_successful_action():
    """Called when a maneuver fully completes. current_node was already advanced
    when the junction was reached; here we finalize the new heading."""
    global current_heading
    if pending_target_direction is not None:
        current_heading = pending_target_direction


def build_action_preview():
    """Human-readable preview of the whole remaining plan as (from,to,dir,action)."""
    if not planned_path or len(planned_path) < 2:
        return []
    preview = []
    h = current_heading
    for k in range(len(planned_path) - 1):
        n1, n2 = planned_path[k], planned_path[k + 1]
        d = get_direction_between(n1, n2)
        if d is None:
            break
        a = direction_to_action(h, d)
        preview.append((n1, n2, d, a))
        h = d
    preview.append((planned_path[-1], None, None, "STOP"))
    return preview


def choose_start_and_destination():
    """Ask for start node, heading, and destination at startup / on reset."""
    node = input("Start node: ").strip().upper()
    while node not in GRAPH:
        print(f"Available nodes: {list(GRAPH.keys())}")
        node = input("Invalid node. Start node: ").strip().upper()

    heading = input("Heading (NORTH/EAST/SOUTH/WEST): ").strip().upper()
    while heading not in DIRECTIONS:
        heading = input(f"Invalid. Heading {DIRECTIONS}: ").strip().upper()
    if heading not in GRAPH.get(node, {}):
        print(f"[WARN] {node} has no road going {heading}; "
              f"available: {list(GRAPH[node].keys())}. Localization may be wrong.")

    dest = input("Destination (usually A/B/C): ").strip().upper()
    while dest not in GRAPH:
        print(f"Available nodes: {list(GRAPH.keys())}")
        dest = input("Invalid node. Destination: ").strip().upper()

    return node, heading, dest


def start_navigation(now):
    """Decide and start the FIRST maneuver out of current_node (used at startup
    and whenever the destination/start is changed)."""
    global state, state_start, pending_next_node, pending_target_direction
    global pending_turn_angle, target_yaw, no_path
    global one_line_counter, no_line_counter

    one_line_counter = no_line_counter = 0
    reset_pid()
    reset_lane_memory()

    if planned_path is None:
        no_path = True
        state = STATE_NO_PATH
        state_start = now
        print("[NAV] NO PATH TO DESTINATION")
        return

    if current_node == destination:
        state = STATE_DONE
        state_start = now
        print(f"[NAV] already at destination {destination}")
        return

    next_node = planned_path[1]
    target_direction = get_direction_between(current_node, next_node)
    action = direction_to_action(current_heading, target_direction)
    pending_next_node = next_node
    pending_target_direction = target_direction

    if action == "STRAIGHT":
        # aligned with the first edge: just follow the line
        state = STATE_FOLLOW
        state_start = now
        print(f"[NAV] start @{current_node} heading {current_heading}; "
              f"STRAIGHT -> FOLLOW toward {next_node}")
    else:
        # misaligned start: turn in place to face the first edge (no forward gap)
        if action == "LEFT":
            pending_turn_angle = LEFT_TURN_ANGLE
        elif action == "RIGHT":
            pending_turn_angle = RIGHT_TURN_ANGLE
        else:
            pending_turn_angle = UTURN_ANGLE
        target_yaw = wrap_angle(target_yaw + pending_turn_angle)
        state = STATE_TURN
        state_start = now
        print(f"[NAV] start @{current_node}; {action} -> align TURN to "
              f"{target_direction} toward {next_node}")


def full_reset(now, node=None, heading=None, dest=None):
    """Restart navigation from a clean slate WITHOUT relaunching the program.
    Clears all blocked edges / sign rules / recovery state and re-plans. Uses the
    given start/heading/dest, or the values entered when the program first started."""
    global current_node, current_heading, destination, previous_node
    global target_yaw, pending_turn_angle, pending_uturn_retreat_after_turn
    global one_line_counter, no_line_counter, forward_gap_time
    global forced_turn_rule
    global yolo_last_forbidden_cmd, yolo_last_forbidden_time, yolo_rule_status
    global yolo_last_forced_cmd, yolo_last_forced_time

    current_node = node if node is not None else init_node
    current_heading = heading if heading is not None else init_heading
    destination = dest if dest is not None else init_dest

    blocked_edges.clear()
    forbidden_rules.clear()
    forced_turn_rule = None
    previous_node = None
    target_yaw = 0.0
    pending_turn_angle = RIGHT_TURN_ANGLE
    pending_uturn_retreat_after_turn = False
    one_line_counter = no_line_counter = 0
    forward_gap_time = FORWARD_GAP_TIME
    yolo_last_forbidden_cmd = None
    yolo_last_forbidden_time = 0.0
    yolo_last_forced_cmd = None
    yolo_last_forced_time = 0.0
    yolo_rule_status = "NONE"

    reset_pid()
    reset_lane_memory()
    plan_route()
    start_navigation(now)
    print(f"[RESET] full reset -> start={current_node} "
          f"heading={current_heading} dest={destination}")


def try_nopath_uturn_recovery(now, arrived):
    """No route forward from `arrived`. Instead of stopping, physically U-turn,
    retreat to the node we came from (previous_node), and reroute from there.

    Spinning + replanning from the SAME node can't find a new path (Dijkstra is
    heading-independent and already considers the reverse edge); what helps is
    moving back to a node that still reaches the destination. We:
      1. block previous_node -> arrived so we never re-enter this dead end,
      2. re-run Dijkstra FROM previous_node,
      3. if a path exists, build [arrived -> previous_node -> ...] so the first
         move is an in-place 180, then the car moves back along the road,
      4. only truly stop (STATE_NO_PATH) if previous_node can't reach it either.
    Sets `state` and returns a status string."""
    global planned_path, no_path, state, state_start
    global pending_turn_angle, pending_target_direction, pending_next_node, target_yaw
    global pending_uturn_retreat_after_turn

    if (previous_node is None or previous_node == arrived
            or current_heading not in DIRECTIONS):
        no_path = True
        state = STATE_NO_PATH
        state_start = now
        return f"[{arrived}] NO PATH (nowhere to retreat) -> STOPPED"

    # 1) Never route back through the dead-end node.
    d_into = get_direction_between(previous_node, arrived)
    if d_into is not None:
        blocked_edges.add((previous_node, d_into))

    # 2) Re-plan from the node behind us.
    path_from_prev = dijkstra(GRAPH, previous_node, destination, blocked_edges)
    if path_from_prev is None:
        no_path = True
        state = STATE_NO_PATH
        state_start = now
        return f"[{arrived}] NO PATH from {previous_node} either -> STOPPED"

    # 3) Plan = retreat (arrived -> previous_node) + shortest path from previous_node.
    planned_path = [arrived] + path_from_prev
    no_path = False

    # 4) The retreat is a 180 in place; afterwards we face the opposite way.
    reverse_heading = DIRECTIONS[(DIRECTIONS.index(current_heading) + 2) % 4]
    pending_next_node = previous_node
    pending_target_direction = reverse_heading
    pending_turn_angle = UTURN_ANGLE
    pending_uturn_retreat_after_turn = True
    target_yaw = wrap_angle(target_yaw + pending_turn_angle)
    reset_pid()
    state = STATE_TURN
    state_start = now
    print(f"[RECOVER] NO PATH @{arrived}: TURN 180 first, then retreat to {previous_node}, "
          f"new plan {planned_path}")
    return f"[{arrived}] NO PATH -> TURN 180 THEN BACKWARD via {previous_node}: {path_from_prev}"


def on_junction_reached(now, jtype):
    """Called when a junction is CONFIRMED. The car has physically reached the
    next node on the plan. We advance localization, honor closed-road rules
    (block + replan), then decide and start the maneuver to leave this node."""
    global current_node, planned_path
    global pending_next_node, pending_target_direction, pending_turn_angle
    global pending_uturn_retreat_after_turn
    global state, state_start, no_path, target_yaw, previous_node
    global forced_turn_rule

    if planned_path is None or len(planned_path) < 2:
        state = STATE_DONE
        state_start = now
        return f"[{jtype}] no plan / already done"

    # 1) Arrival: current_node advances to the node we just reached.
    previous_node = planned_path[0]   # remember where we came from (for U-turn recovery)
    arrived = planned_path[1]
    current_node = arrived
    planned_path = planned_path[1:]   # keeps invariant planned_path[0] == current_node

    # 2) Reached the destination -> STOP.
    if current_node == destination:
        state = STATE_DONE
        state_start = now
        return f"[{jtype}] ARRIVED AT DESTINATION {destination} -> STOP"

    # 3) Closed-road handling BEFORE committing to a turn.
    if forbidden_rules:
        if block_forbidden_edges(current_node, current_heading):
            if not replan_after_block():
                forbidden_rules.clear()
                return try_nopath_uturn_recovery(now, current_node)
        print(f"[RULE] consumed {sorted(forbidden_rules)} @{current_node}")
        forbidden_rules.clear()

    # 4) Forced turn handling (YOLO SAG/SOL) BEFORE committing to normal Dijkstra action.
    force_msg = apply_forced_turn_rule(now)
    if force_msg:
        print(force_msg)

    # 5) Safety: after replanning / forced-turn handling we must have a next node.
    if planned_path is None or len(planned_path) < 2:
        if current_node == destination:
            state = STATE_DONE
            state_start = now
            return f"[{jtype}] DESTINATION {destination} -> STOP"
        return try_nopath_uturn_recovery(now, current_node)

    # 6) Decide the action that leaves current_node toward the next node.
    next_node = planned_path[1]
    target_direction = get_direction_between(current_node, next_node)
    action = direction_to_action(current_heading, target_direction)
    pending_next_node = next_node
    pending_target_direction = target_direction

    if action == "STRAIGHT":
        reset_pid()
        state = STATE_STRAIGHT_PASS
        state_start = now
        return f"@{current_node} STRAIGHT -> PASS (hdg {current_heading}, next {next_node})"
    elif action == "LEFT":
        pending_turn_angle = LEFT_TURN_ANGLE
        reset_pid()
        state = STATE_FORWARD_GAP
        state_start = now
        return f"@{current_node} LEFT -> turn {target_direction} (next {next_node})"
    elif action == "RIGHT":
        pending_turn_angle = RIGHT_TURN_ANGLE
        reset_pid()
        state = STATE_FORWARD_GAP
        state_start = now
        return f"@{current_node} RIGHT -> turn {target_direction} (next {next_node})"
    else:  # UTURN -> spin 180 FIRST, then drive backward
        pending_turn_angle = UTURN_ANGLE
        pending_uturn_retreat_after_turn = True
        target_yaw = wrap_angle(target_yaw + pending_turn_angle)
        reset_pid()
        state = STATE_TURN
        state_start = now
        return f"@{current_node} UTURN -> 180 first, then go back to {target_direction} (next {next_node})"


def set_forbidden_turn(value):
    """Add one forbidden rule, or clear all with None. Combos allowed (l then f)."""
    global forbidden_rules
    if value is None:
        forbidden_rules.clear()
        print("[RULE] cleared. Normal driving.")
        return
    forbidden_rules.add(value)
    print(f"[RULE] active forbidden rules: {sorted(forbidden_rules)}")


def set_forced_turn(value):
    """Store a one-shot forced turn from YOLO (SAG/SOL). It is consumed at
    the next confirmed junction. Latest sign wins if SAG and SOL appear before
    the same junction."""
    global forced_turn_rule
    if value is None:
        forced_turn_rule = None
        print("[FORCE] cleared.")
        return
    if value in ("LEFT", "RIGHT"):
        forced_turn_rule = value
        print(f"[FORCE] next junction forced turn: {forced_turn_rule}")


# YOLO closed-road action -> relative forbidden rule + dashboard status text.
#   SAGKAPALI   (sagadonulmez) -> RIGHT closed
#   SOLKAPALI   (soladonulmez) -> LEFT  closed
#   ILERIKAPALI (girisyok)     -> FORWARD closed  (road ahead is no-entry)
YOLO_FORBIDDEN_MAP = {
    "SAGKAPALI":   ("RIGHT",   "YOLO: RIGHT FORBIDDEN"),
    "SOLKAPALI":   ("LEFT",    "YOLO: LEFT FORBIDDEN"),
    "ILERIKAPALI": ("FORWARD", "YOLO: FORWARD FORBIDDEN"),
}

# YOLO mandatory-turn action -> relative forced turn consumed at next junction.
#   SAG -> force RIGHT at the next junction, then replan from the chosen branch.
#   SOL -> force LEFT  at the next junction, then replan from the chosen branch.
YOLO_FORCE_TURN_MAP = {
    "SAG": ("RIGHT", "YOLO: FORCE RIGHT"),
    "SOL": ("LEFT",  "YOLO: FORCE LEFT"),
}


def handle_yolo_forbidden(action_label, conf):
    """PC YOLO action results -> one-shot navigation rules.

    Closed-road signs (SAGKAPALI / SOLKAPALI / ILERIKAPALI) are consumed at the
    next junction as blocked edges. Direction signs (SAG / SOL) are consumed at
    the next junction as a forced right/left choice; after that edge is selected,
    the graph planner recalculates the route from the branch it entered.
    """
    global yolo_last_forbidden_cmd, yolo_last_forbidden_time, yolo_rule_status
    global yolo_last_forced_cmd, yolo_last_forced_time

    cmd = (action_label or "").strip().upper()
    now = time.time()

    # Mandatory direction signs: force next junction turn.
    if cmd in YOLO_FORCE_TURN_MAP and conf >= YOLO_FORBIDDEN_CONF:
        if (cmd == yolo_last_forced_cmd and
                now - yolo_last_forced_time < yolo_forced_cooldown):
            return
        rule, status = YOLO_FORCE_TURN_MAP[cmd]
        set_forced_turn(rule)
        yolo_rule_status = status
        yolo_last_forced_cmd = cmd
        yolo_last_forced_time = now
        return

    # Closed-road / forbidden signs: block a relative edge at the next junction.
    if cmd in YOLO_FORBIDDEN_MAP and conf >= YOLO_FORBIDDEN_CONF:
        if (cmd == yolo_last_forbidden_cmd and
                now - yolo_last_forbidden_time < yolo_forbidden_cooldown):
            return
        rule, status = YOLO_FORBIDDEN_MAP[cmd]
        set_forbidden_turn(rule)
        yolo_rule_status = status
        yolo_last_forbidden_cmd = cmd
        yolo_last_forbidden_time = now
        return

    # No active YOLO navigation sign in this frame. Keep pending one-shot rules,
    # but reset cooldown identities so a newly seen sign can be accepted later.
    yolo_last_forbidden_cmd = None
    yolo_last_forced_cmd = None
    if forced_turn_rule is not None:
        yolo_rule_status = f"PENDING FORCE {forced_turn_rule}"
    elif forbidden_rules:
        yolo_rule_status = f"PENDING FORBID {sorted(forbidden_rules)}"
    else:
        yolo_rule_status = "NONE"


# ======================================================================
# VISION (unchanged behavior)
# ======================================================================
def get_mask(roi):
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = clahe.apply(l)
    roi_c = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    hsv = cv2.cvtColor(roi_c, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_pink, upper_pink))
    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_blue, upper_blue))
    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_green, upper_green))
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def analyze_band(mask, y):
    h, w = mask.shape
    cx = w // 2
    y = clamp(y, 0, h - 1)
    band = mask[clamp(y - BAND_HALF, 0, h - 1):clamp(y + BAND_HALF, 0, h), :]
    Ml = cv2.moments(band[:, :cx])
    Mr = cv2.moments(band[:, cx:])
    lf = Ml["m00"] > 500
    rf = Mr["m00"] > 500
    lc = int(Ml["m10"] / Ml["m00"]) if lf else None
    rc = int(Mr["m10"] / Mr["m00"]) + cx if rf else None
    if lf and rf:
        road_c = (lc + rc) // 2; conf = 1.0; found = True
    elif lf:
        road_c = lc + ROAD_HALF_WIDTH; conf = 0.55; found = True
    elif rf:
        road_c = rc - ROAD_HALF_WIDTH; conf = 0.55; found = True
    else:
        road_c = cx; conf = 0.0; found = False
    return {"y": y, "found": found, "left_found": lf, "right_found": rf,
            "left_center": lc, "right_center": rc,
            "road_center": road_c, "confidence": conf}


def measure_side_strength(mask, y, side):
    h, w = mask.shape
    cx = w // 2
    y1 = clamp(y - ONE_LINE_SCAN_HALF_H, 0, h - 1)
    y2 = clamp(y + ONE_LINE_SCAN_HALF_H, 0, h)
    roi = mask[y1:y2, :cx] if side == "left" else mask[y1:y2, cx:]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, 0
    biggest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(biggest)
    _, _, _, bh = cv2.boundingRect(biggest)
    return area, bh


def detect_road(frame):
    global smooth_error, last_road_center
    roi = frame[ROI_TOP:ROI_BOTTOM, :]
    mask = get_mask(roi)
    bands = [analyze_band(mask, y) for y in BANDS_Y]
    cx = CAM_W // 2

    found_bands = [b for b in bands if b["found"]]
    if found_bands:
        weights = [0.25, 0.35, 0.80]
        tw = tv = 0.0
        for b, w in zip(bands, weights):
            if b["found"]:
                rw = w * b["confidence"]
                tv += b["road_center"] * rw
                tw += rw
        road_target = int(tv / tw)
        last_road_center = road_target
        line_found = True
    else:
        road_target = last_road_center
        line_found = False

    raw_error = road_target - cx
    smooth_error = ERROR_ALPHA * smooth_error + (1.0 - ERROR_ALPHA) * raw_error

    dotY1 = bands[0]
    dotY2 = bands[1]
    dotX = bands[2]

    left_area, left_len = measure_side_strength(mask, dotY2["y"], "left")
    right_area, right_len = measure_side_strength(mask, dotY2["y"], "right")

    valid_left_line = (dotY2["left_found"] and left_area >= MIN_ONE_LINE_AREA and left_len >= MIN_ONE_LINE_LEN)
    valid_right_line = (dotY2["right_found"] and right_area >= MIN_ONE_LINE_AREA and right_len >= MIN_ONE_LINE_LEN)

    two_lines = dotX["left_found"] and dotX["right_found"]

    only_left = valid_left_line and not dotY2["right_found"] and not dotY1["right_found"]
    only_right = valid_right_line and not dotY2["left_found"] and not dotY1["left_found"]
    no_line = not dotY2["left_found"] and not dotY2["right_found"]

    debug = frame.copy()
    cv2.line(debug, (cx, 0), (cx, CAM_H), (255, 0, 0), 2)
    cv2.line(debug, (cx - CENTER_TOL, 0), (cx - CENTER_TOL, CAM_H), (0, 255, 0), 1)
    cv2.line(debug, (cx + CENTER_TOL, 0), (cx + CENTER_TOL, CAM_H), (0, 255, 0), 1)

    for idx, b in enumerate(bands):
        col = [(255, 0, 255), (0, 255, 255), (200, 200, 0)][idx]
        name = ["dotY1", "dotY2 ONE", "dotX TWO"][idx]
        cv2.line(debug, (0, b["y"]), (CAM_W, b["y"]), col, 1)
        cv2.putText(debug, name, (CAM_W - 120, b["y"] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        if b["left_center"] is not None:
            cv2.circle(debug, (b["left_center"], b["y"]), 6, (0, 0, 255), -1)
        if b["right_center"] is not None:
            cv2.circle(debug, (b["right_center"], b["y"]), 6, (255, 0, 0), -1)
        if b["found"]:
            cv2.circle(debug, (b["road_center"], b["y"]), 8, (0, 255, 0), -1)

    cv2.circle(debug, (road_target, BANDS_Y[1]), 12, (0, 255, 255), -1)
    cv2.line(debug, (road_target, 0), (road_target, CAM_H), (0, 255, 255), 2)

    return {
        "error": smooth_error, "raw_error": raw_error, "line_found": line_found,
        "two_lines": two_lines, "only_left": only_left,
        "only_right": only_right, "no_line": no_line,
        "left_area": left_area, "left_len": left_len,
        "right_area": right_area, "right_len": right_len,
        "road_target": road_target, "bands": bands,
        "debug": debug, "mask": mask,
    }


def side_controller(error, line_found):
    if not line_found:
        return 0, "AFTER TURN: NO LINE - WAIT"
    if abs(error) <= CENTER_TOL:
        return 0, "AFTER TURN: SIDE CENTERED"
    vy = int(KP_X * error)
    vy = clamp(vy, -MAX_STRAFE, MAX_STRAFE)
    if vy > 0:
        vy = max(vy, MIN_STRAFE)
        return vy, "AFTER TURN: STRAFE LEFT"
    else:
        vy = min(vy, -MIN_STRAFE)
        return vy, "AFTER TURN: STRAFE RIGHT"


def state_name(s):
    return ["FOLLOW", "FORWARD_GAP", "TURN", "AFTER_TURN_SIDE",
            "STRAIGHT_PASS", "DONE", "NO_PATH", "UTURN_BACK"][s]


def apply_web_command(cmd, ser, now):
    """Apply one control dict coming from the web dashboard. Called ONLY from the
    main loop (never the HTTP thread), so it can safely touch navigation globals.

    Supported commands:
      {"cmd": "set_route", "start": "C", "heading": "SOUTH", "dest": "B"}
      {"cmd": "set_dest",  "dest": "B"}
      {"cmd": "reset"}
      {"cmd": "forbid",    "rule": "LEFT" | "RIGHT" | "FORWARD" | "CLEAR"}
    """
    global destination, init_node, init_heading, init_dest

    if not isinstance(cmd, dict):
        return
    c = str(cmd.get("cmd", "")).lower()

    if c == "set_route":
        node = str(cmd.get("start", "")).upper()
        heading = str(cmd.get("heading", "")).upper()
        dest = str(cmd.get("dest", "")).upper()
        if node in GRAPH and heading in DIRECTIONS and dest in GRAPH:
            init_node, init_heading, init_dest = node, heading, dest
            send_command(ser, vx=0, vy=0, wz=0,
                         target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
            full_reset(now, node, heading, dest)
            print(f"[WEB] set_route start={node} heading={heading} dest={dest}")
        else:
            print(f"[WEB] invalid set_route: {cmd}")

    elif c == "set_dest":
        dest = str(cmd.get("dest", "")).upper()
        if dest in GRAPH:
            destination = dest
            send_command(ser, vx=0, vy=0, wz=0,
                         target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
            plan_route()
            start_navigation(now)
            print(f"[WEB] set_dest {dest}")
        else:
            print(f"[WEB] invalid dest: {dest}")

    elif c == "reset":
        send_command(ser, vx=0, vy=0, wz=0,
                     target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
        full_reset(now)
        print("[WEB] reset to original start/heading/dest")

    elif c == "forbid":
        rule = str(cmd.get("rule", "")).upper()
        if rule == "CLEAR":
            set_forbidden_turn(None)
        elif rule in ("LEFT", "RIGHT", "FORWARD"):
            set_forbidden_turn(rule)
        else:
            print(f"[WEB] unknown forbid rule: {rule}")

    else:
        print(f"[WEB] unknown command: {cmd}")


# ==========================================================================
# OBSTACLE ("ENGEL") DETECTION  --  stop-gate layer over the state machine.
# Looks INSIDE the lane band for a foreign blob (not lane colour, not floor).
# If one is confirmed, the main loop sends a hard stop and freezes its timers;
# when the lane clears again, navigation resumes from exactly where it paused.
# ==========================================================================
OBST_ROI_TOP    = int(CAM_H * 0.35)   # top of the area the car watches for engel
OBST_ROI_BOTTOM = int(CAM_H * 0.80)   # bottom of that area
OBST_BAND_MARGIN = 40                 # pixels stepped inside the band edges
OBST_MIN_AREA    = 5000               # blob bigger than this inside the lane = engel

# debounce so a single noisy frame can't start/stop the car
OBSTACLE_CONFIRM_FRAMES = 3           # consecutive frames needed to flip state


def obstacle_lane_mask(roi):
    """Red+blue lane mask for the obstacle ROI (same colours as the road)."""
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = clahe.apply(l)
    roi_c = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    hsv   = cv2.cvtColor(roi_c, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                       cv2.inRange(hsv, lower_red2, upper_red2)),
        cv2.inRange(hsv, lower_blue, upper_blue))
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def detect_obstacle(frame):
    """Return (engel_var, max_area, debug_roi, lane_mask).
    A foreign object inside the lane band (not lane colour, not light floor)
    larger than OBST_MIN_AREA counts as an obstacle."""
    roi  = frame[OBST_ROI_TOP:OBST_ROI_BOTTOM, :]
    h, w = roi.shape[:2]

    lane_mask = obstacle_lane_mask(roi)

    # columns where the lane band is present -> drive corridor
    col_sum   = np.sum(lane_mask, axis=0)
    lane_cols = np.where(col_sum > 0)[0]
    if len(lane_cols) < 10:
        return False, 0, roi.copy(), lane_mask, []

    left_x  = max(0,   int(lane_cols.min()) + OBST_BAND_MARGIN)
    right_x = min(w - 1, int(lane_cols.max()) - OBST_BAND_MARGIN)
    if right_x <= left_x:
        return False, 0, roi.copy(), lane_mask, []

    # inside the corridor: everything that is NOT lane colour
    road_region = lane_mask[:, left_x:right_x]
    not_lane    = cv2.bitwise_not(road_region)

    # also drop the light floor (low saturation, high value)
    roi_crop   = roi[:, left_x:right_x]
    hsv_crop   = cv2.cvtColor(roi_crop, cv2.COLOR_BGR2HSV)
    floor_mask = cv2.inRange(hsv_crop,
                             np.array([0,   0, 120]),
                             np.array([180, 60, 255]))
    engel_mask = cv2.bitwise_and(not_lane, cv2.bitwise_not(floor_mask))

    k2 = np.ones((5, 5), np.uint8)
    engel_mask = cv2.morphologyEx(engel_mask, cv2.MORPH_OPEN,  k2)
    engel_mask = cv2.morphologyEx(engel_mask, cv2.MORPH_CLOSE, k2)

    contours, _ = cv2.findContours(engel_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    engel_var = False
    max_area  = 0
    boxes     = []                     # (x, y, w, h) in FULL-FRAME coordinates
    debug     = roi.copy()
    cv2.line(debug, (left_x, 0),  (left_x, h),  (0, 255, 0), 1)
    cv2.line(debug, (right_x, 0), (right_x, h), (0, 255, 0), 1)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > OBST_MIN_AREA:
            engel_var = True
            max_area  = max(max_area, area)
            x, y, bw, bh = cv2.boundingRect(cnt)
            cv2.rectangle(debug, (x + left_x, y),
                          (x + left_x + bw, y + bh), (0, 0, 255), 2)
            # ROI sits at x=left_x, y=OBST_ROI_TOP in the full frame
            boxes.append((x + left_x, y + OBST_ROI_TOP, bw, bh))

    return engel_var, max_area, debug, engel_mask, boxes


def main():
    global state, state_start
    global no_line_counter, one_line_counter
    global target_yaw, pending_turn_angle, pending_uturn_retreat_after_turn
    global forward_gap_time
    global latest_frame, yolo_running
    global current_node, current_heading, destination
    global previous_node, init_node, init_heading, init_dest

    # ---- start node + heading + destination ----
    node, heading, dest = choose_start_and_destination()
    current_node = node
    current_heading = heading
    destination = dest
    init_node = node            # remember for resets
    init_heading = heading
    init_dest = dest
    plan_route()
    start_navigation(time.time())

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "BGR888"}))
    picam2.start()
    time.sleep(1.0)

    dashboard.set_graph(GRAPH)
    web_cmd_queue = queue.Queue()
    dashboard.set_control_callback(lambda d: web_cmd_queue.put(d))
    dashboard.start(port=8080)
    print("route_nav.py (graph navigation) started")
    print("keys:")
    print("  a/b/c = set destination A/B/C and replan from current node")
    print("  0 = RESET (restart from the original start/heading/dest, clears everything)")
    print("  r = reset AND choose new start node + heading + destination")
    print("  l = LEFT forbidden   t = RIGHT forbidden")
    print("  f = FORWARD forbidden  n = clear forbidden rules")
    print("  q = quit")
    print("YOLO: SAGKAPALI=RIGHT forbidden, SOLKAPALI=LEFT forbidden, "
          "ILERIKAPALI=FORWARD forbidden (turns around if that's the only route)")

    threading.Thread(target=yolo_thread_fn, daemon=True).start()

    os.makedirs("/home/ehsan/carcodes/recordings", exist_ok=True)
    rec_name = time.strftime("/home/ehsan/carcodes/recordings/rec_%Y%m%d_%H%M%S.avi")
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    writer = cv2.VideoWriter(rec_name, fourcc, 20.0, (CAM_W * 2, CAM_H))
    print(f"Recording: {rec_name}")

    t_prev = time.time()

    # ----- obstacle ("engel") stop-gate state -----
    obstacle_active = False     # True while the car is held by an obstacle
    obstacle_seen_cnt = 0       # consecutive frames an obstacle was seen
    obstacle_clear_cnt = 0      # consecutive frames the lane was clear

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            with latest_frame_lock:
                latest_frame = frame.copy()

            road = detect_road(frame)
            two = road["two_lines"]
            only_left = road["only_left"]
            only_right = road["only_right"]
            no_line = road["no_line"]
            err = road["error"]
            raw_err = road["raw_error"]
            line_found = road["line_found"]
            debug = road["debug"]
            mask = road["mask"]

            now = time.time()
            dt = max(now - t_prev, 0.001)
            fps = 1.0 / dt
            t_prev = now

            # ----- apply any commands queued from the web dashboard -----
            while True:
                try:
                    wcmd = web_cmd_queue.get_nowait()
                except queue.Empty:
                    break
                apply_web_command(wcmd, ser, now)

            with yolo_lock:
                ylabel = yolo_label
                yconf = yolo_conf

            handle_yolo_forbidden(ylabel, yconf)

            # ----- obstacle ("engel") detection + debounce -----
            engel, engel_area, engel_dbg, engel_mask, engel_boxes = detect_obstacle(frame)
            if engel:
                obstacle_seen_cnt += 1
                obstacle_clear_cnt = 0
            else:
                obstacle_clear_cnt += 1
                obstacle_seen_cnt = 0

            if not obstacle_active and obstacle_seen_cnt >= OBSTACLE_CONFIRM_FRAMES:
                obstacle_active = True
                print(f"[ENGEL] obstacle -> STOP (area={engel_area:.0f})")
            elif obstacle_active and obstacle_clear_cnt >= OBSTACLE_CONFIRM_FRAMES:
                obstacle_active = False
                reset_pid()          # avoid a derivative kick after the pause
                print("[ENGEL] clear -> RESUME")

            # Draw the obstacle on the interface (this `debug` frame is what the
            # web dashboard streams, so it must be drawn BEFORE push_frame below).
            if engel:
                for (bx, by, bw, bh) in engel_boxes:
                    cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (0, 0, 255), 3)
                    cv2.putText(debug, "ENGEL", (bx, max(15, by - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(debug, f"ENGEL ALGILANDI (area={engel_area:.0f})",
                            (10, CAM_H - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

            vx = vy = wz = 0
            mode = MODE_DRIVE_HEADING
            action = "STOP"

            na = next_action()
            confirm_one = STOP_CONFIRM_LIMIT if na == "STOP" else ONE_LINE_LIMIT
            confirm_no = STOP_CONFIRM_LIMIT if na == "STOP" else NO_LINE_LIMIT

            if obstacle_active and state not in (STATE_DONE, STATE_NO_PATH):
                # Hold position. Push state_start forward by dt so the elapsed
                # time of any timed state (FORWARD_GAP / TURN / PASS / ...) is
                # frozen and resumes seamlessly once the obstacle clears.
                state_start += dt
                vx = vy = wz = 0
                mode = MODE_DRIVE_HEADING
                action = f"ENGEL STOP area={engel_area:.0f} ({state_name(state)})"

            elif state == STATE_FOLLOW:
                if two:
                    one_line_counter = no_line_counter = 0
                    wz = pid_update(err)
                    vx = FORWARD_SPEED
                    action = f"TWO LINES -> FOLLOW  (next:{na})"

                elif no_line:
                    no_line_counter += 1
                    one_line_counter = 0
                    action = f"BOTH JUNCTION? ({no_line_counter}/{confirm_no}) next:{na}"
                    if no_line_counter >= confirm_no:
                        no_line_counter = 0
                        forward_gap_time = FORWARD_GAP_TIME
                        action = on_junction_reached(now, "BOTH")

                elif only_left or only_right:
                    one_line_counter += 1
                    no_line_counter = 0
                    jtype = "LEFT_ONLY" if only_right else "RIGHT_ONLY"
                    action = f"{jtype} JUNCTION? ({one_line_counter}/{confirm_one}) next:{na}"
                    if one_line_counter >= confirm_one:
                        one_line_counter = 0
                        forward_gap_time = ONE_LINE_GAP_TIME
                        action = on_junction_reached(now, jtype)

                else:
                    one_line_counter = no_line_counter = 0
                    vx = FORWARD_SPEED
                    wz = pid_update(err)
                    action = "WEAK -> FOLLOW"

            elif state == STATE_FORWARD_GAP:
                vx = FORWARD_SPEED
                vy = 0
                wz = 0
                action = f"FORWARD GAP ({forward_gap_time:.2f}s)"
                if now - state_start >= forward_gap_time:
                    target_yaw = wrap_angle(target_yaw + pending_turn_angle)
                    state = STATE_TURN
                    state_start = now
                    action = f"GAP DONE -> TURN target={target_yaw:.1f}"

            elif state == STATE_TURN:
                vx = 0; vy = 0; wz = 0
                mode = MODE_ROTATE_YAW
                action = "YAW TURNING"
                turn_time = UTURN_TIME if abs(pending_turn_angle) > 100 else TURN_TIME
                if now - state_start >= turn_time:
                    reset_pid()
                    reset_lane_memory()
                    one_line_counter = no_line_counter = 0
                    if pending_uturn_retreat_after_turn:
                        # For U-turns: finish the 180 first, then drive backward.
                        # After turning 180, positive vx moves the car toward the previous node.
                        update_position_after_successful_action()
                        state = STATE_UTURN_BACK
                        state_start = now
                        action = "TURN 180 DONE -> BACKWARD"
                    else:
                        state = STATE_AFTER_TURN_SIDE
                        state_start = now
                        action = "TURN DONE -> AFTER_TURN_SIDE"

            elif state == STATE_AFTER_TURN_SIDE:
                if two:
                    if abs(err) <= CENTER_TOL:
                        update_position_after_successful_action()
                        reset_pid()
                        one_line_counter = no_line_counter = 0
                        state = STATE_FOLLOW
                        state_start = now
                        vx = FORWARD_SPEED; vy = 0; wz = 0
                        action = "CENTERED -> FOLLOW"
                    else:
                        vx = 0; wz = 0
                        vy, action = side_controller(err, True)
                        mode = MODE_DRIVE_HEADING
                elif only_left or only_right:
                    vx = 0; wz = 0
                    vy, action = side_controller(err, True)
                    mode = MODE_DRIVE_HEADING
                elif no_line:
                    vx = 0; vy = 0; wz = 0
                    action = "AFTER TURN: NO LINE - WAIT"
                else:
                    vx = 0; wz = 0
                    vy, action = side_controller(err, line_found)
                    mode = MODE_DRIVE_HEADING
                    if abs(err) <= CENTER_TOL and line_found:
                        update_position_after_successful_action()
                        reset_pid()
                        one_line_counter = no_line_counter = 0
                        state = STATE_FOLLOW
                        state_start = now
                        vx = FORWARD_SPEED; vy = 0
                        action = "WEAK LINE CENTERED -> FOLLOW"

            elif state == STATE_UTURN_BACK:
                # U-turn order: rotate 180 first, then send a real BACKWARD command.
                # Negative vx means the car moves backward relative to its new 180-degree heading.
                vx = -UTURN_BACK_SPEED; vy = 0; wz = 0
                mode = MODE_DRIVE_HEADING
                action = f"UTURN BACKWARD ({UTURN_BACK_TIME:.2f}s after 180)"
                if now - state_start >= UTURN_BACK_TIME:
                    pending_uturn_retreat_after_turn = False
                    reset_pid()
                    reset_lane_memory()
                    one_line_counter = no_line_counter = 0
                    state = STATE_FOLLOW
                    state_start = now
                    action = "BACKWARD DONE -> FOLLOW"

            elif state == STATE_STRAIGHT_PASS:
                vx = FORWARD_SPEED; vy = 0; wz = 0
                mode = MODE_DRIVE_HEADING
                action = f"PASS JUNCTION ({STRAIGHT_PASS_TIME:.2f}s)"
                if now - state_start >= STRAIGHT_PASS_TIME:
                    update_position_after_successful_action()
                    reset_pid()
                    reset_lane_memory()
                    one_line_counter = no_line_counter = 0
                    state = STATE_FOLLOW
                    state_start = now
                    action = "PASS DONE -> FOLLOW"

            elif state == STATE_DONE:
                vx = vy = wz = 0
                mode = MODE_DRIVE_HEADING
                action = f"DESTINATION {destination} REACHED - STOPPED"

            elif state == STATE_NO_PATH:
                vx = vy = wz = 0
                mode = MODE_DRIVE_HEADING
                action = "NO PATH TO DESTINATION - STOPPED"

            # ----- dashboard -----
            step = get_next_planned_step()
            dashboard.push_frame(debug)
            dashboard.push_mask(mask)
            dashboard.push_state(
                start=current_node, dest=destination,
                state_name=state_name(state), state_id=state,
                route=list(planned_path) if planned_path else [],
                route_index=0,
                route_selected=(planned_path is not None),
                route_variant=current_heading or '-',
                next_action=na,
                vx=vx, vy=vy, wz=wz,
                error=float(err), raw_error=int(raw_err),
                yaw=float(target_yaw), fps=float(fps),
                two=bool(two), only_left=bool(only_left),
                only_right=bool(only_right), no_line=bool(no_line),
                yolo_label=ylabel, yolo_conf=float(yconf),
                yolo_rule=yolo_rule_status,
                forbidden=sorted(forbidden_rules),
                forced_turn=forced_turn_rule,
                one_line_cnt=one_line_counter,
                no_line_cnt=no_line_counter,
                # ----- graph navigation fields -----
                current_node=current_node,
                current_heading=current_heading,
                destination=destination,
                path=list(planned_path) if planned_path else [],
                next_node=step.get("next_node"),
                target_direction=step.get("target_direction"),
                blocked_edges=sorted([f"{n}:{d}" for (n, d) in blocked_edges]),
                no_path=bool(no_path),
                # ----- obstacle ("engel") status -----
                obstacle=bool(obstacle_active),
                obstacle_seen=bool(engel),
                obstacle_area=float(engel_area),
            )

            send_command(ser, vx=vx, vy=vy, wz=wz,
                         target_yaw=target_yaw, mode=mode)

            # ----- debug overlay -----
            ptxt = " ".join([f"{a}@{n1}" for (n1, n2, d, a) in build_action_preview()])
            cv2.putText(debug, f"NODE={current_node} HDG={current_heading} DEST={destination}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.putText(debug, f"PATH={planned_path if planned_path else '(none)'}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(debug, f"NEXT={step.get('next_node')} DIR={step.get('target_direction')} ACT={na}",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(debug, f"STATE={state_name(state)}  ACTION={action}",
                        (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(debug, f"raw={raw_err:+d} smooth={err:+.1f} vx={vx} vy={vy} wz={wz}",
                        (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(debug, f"two={two} L={only_left} R={only_right} no={no_line}  FPS={fps:.1f}",
                        (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(debug, f"turn={pending_turn_angle:+.0f} yaw={target_yaw:.1f}",
                        (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 255), 1)
            cv2.putText(debug, f"YOLO={ylabel} ({yconf:.2f})  RULE={yolo_rule_status}",
                        (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(debug, f"FORBIDDEN={sorted(forbidden_rules)} FORCE={forced_turn_rule}",
                        (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1)
            cv2.putText(debug, f"BLOCKED={[f'{n}:{d}' for (n,d) in sorted(blocked_edges)]}",
                        (10, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 255), 1)
            if ptxt:
                cv2.putText(debug, f"PLAN={ptxt}",
                            (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1)

            cv2.imshow("engel", engel_dbg)

            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack([debug, mask_bgr])
            writer.write(combined)

            cv2.imshow("route_nav debug", debug)
            cv2.imshow("mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            elif key in (ord("a"), ord("b"), ord("c")):
                new_dest = chr(key).upper()
                if new_dest in GRAPH:
                    destination = new_dest
                    send_command(ser, vx=0, vy=0, wz=0,
                                 target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
                    plan_route()
                    start_navigation(time.time())
                else:
                    print(f"[WARN] {new_dest} is not a node")

            elif key == ord("0"):
                # full reset: stop, clear everything, restart from the ORIGINAL
                # start/heading/dest entered at launch (no re-typing needed).
                send_command(ser, vx=0, vy=0, wz=0,
                             target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
                full_reset(time.time())

            elif key == ord("r"):
                # reset AND enter fresh start/heading/dest from the terminal.
                send_command(ser, vx=0, vy=0, wz=0,
                             target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
                node, heading, dest = choose_start_and_destination()
                full_reset(time.time(), node, heading, dest)

            elif key == ord("l"):
                set_forbidden_turn("LEFT")
            elif key == ord("t"):
                set_forbidden_turn("RIGHT")
            elif key == ord("f"):
                set_forbidden_turn("FORWARD")
            elif key == ord("n"):
                set_forbidden_turn(None)
                set_forced_turn(None)

    finally:
        print("Stopping route_nav.py")
        yolo_running = False
        time.sleep(0.2)
        writer.release()
        print(f"Video saved: {rec_name}")
        send_command(ser, vx=0, vy=0, wz=0,
                     target_yaw=target_yaw, mode=MODE_DRIVE_HEADING)
        picam2.stop()
        cv2.destroyAllWindows()
        ser.close()


if __name__ == "__main__":
    main()