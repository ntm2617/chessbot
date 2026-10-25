import ikpy.chain
import numpy as np
from controller import Robot, Supervisor
import warnings
import os
import re

# ==============================================================================
# 1. MOVE QUEUE (Shared Game Schedule)
# ==============================================================================
# White pieces are on ranks 1-4  -> Handled by White Robot (UR10e(1))
# Black pieces are on ranks 5-8  -> Handled by Black Robot (UR10e)
move_queue = [
    ("e8", "a1"),   # Black test move
    # ("e2", "e4"), # White move
    # ("e7", "e5"), # Black move
]

# Height settings:
Z_TRAVEL = 0.50        # High cruise height across board
Z_CLEAR  = 0.30        # High overhead clearance (lifts piece 7cm above all pawn heads)


# ==============================================================================
# 2. CHESSBOARD 2-POINT GROUND TRUTH CALIBRATION & PIECE HEIGHTS
# ==============================================================================
A1_X, A1_Y = -0.41, 0.38
H8_X, H8_Y = -1.15, -0.38

def square_to_world(square_str):
    """Calculates the exact (X, Y) world coordinates of any square a1 to h8."""
    sq = square_str.strip().lower()
    col = ord(sq[0]) - ord('a')   # 'a'->0, 'h'->7
    row = int(sq[1]) - 1          # '1'->0, '8'->7
    
    x = A1_X + (col / 7.0) * (H8_X - A1_X)
    y = A1_Y + (row / 7.0) * (H8_Y - A1_Y)
    return np.array([x, y])

def get_piece_touch_z(square_str):
    """Returns the exact top touchdown height for each piece type in Webots."""
    sq = square_str.strip().lower()
    col = sq[0]
    rank = int(sq[1])
    
    if rank in {2, 7}:
        return 0.145  # Pawns
    if col in {'b', 'g'}:
        return 0.173  # Knights
    elif col in {'e'}:
        return 0.178  # Kings
    else:
        return 0.165  # Bishops, Rooks, Queens

# ==============================================================================
# 3. ROBOT & VACUUM GRIPPER INITIALIZATION
# ==============================================================================
try:
    robot = Supervisor()
    is_supervisor = robot.getSelf() is not None
except Exception:
    robot = Robot()
    is_supervisor = False

timestep = int(robot.getBasicTimeStep())
robot_name = robot.getName()

# Dual-Robot Role Identification:
is_white_robot = ("(1)" in robot_name) or ("white" in robot_name.lower())
is_black_robot = not is_white_robot

if is_white_robot:
    role_str = "WHITE ROBOT [UR10e(1)]"
    my_ranks = {1, 2, 3, 4}
else:
    role_str = "BLACK ROBOT [UR10e]"
    my_ranks = {5, 6, 7, 8}

def is_my_move(square_str):
    """Returns True if the piece on square_str belongs to this robot's territory."""
    try:
        rank = int(square_str.strip()[-1])
        return rank in my_ranks
    except Exception:
        return False

# Arm motor initialization
arm_motor_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
arm_motors = [robot.getDevice(name) for name in arm_motor_names]
for motor in arm_motors:
    motor.setVelocity(0.35)
    motor.setAcceleration(0.7)

def calc_duration(angles_a, angles_b, v=0.35):
    """Calculates physical flight time required between two joint configurations."""
    a = angles_a[1:7] if len(angles_a) > 6 else angles_a
    b = angles_b[1:7] if len(angles_b) > 6 else angles_b
    diffs = [abs(x - y) for x, y in zip(a, b)]
    return (max(diffs) / v) + 0.8

# Replace lines 108-112 with:
current_arm_angles = [-1.571, -1.189, 1.805, 0.955, 1.571, 0.0]

def set_arm_angles(angles, v=0.35):
    """Sets joint targets with synchronized speeds so all joints move together without sagging in mid-air."""
    global current_arm_angles
    target = angles[1:7] if len(angles) > 6 else angles
    diffs = [abs(t - c) for t, c in zip(target, current_arm_angles)]
    max_diff = max(diffs)
    
    if max_diff > 1e-4:
        for motor, target_val, diff in zip(arm_motors, target, diffs):
            motor_v = max(0.02, v * (diff / max_diff))
            motor.setVelocity(motor_v)
            motor.setPosition(target_val)
    else:
        for motor, target_val in zip(arm_motors, target):
            motor.setVelocity(v)
            motor.setPosition(target_val)
            
    current_arm_angles = list(target)

# Vacuum gripper device
vacuum = robot.getDevice("vacuum gripper")
if vacuum:
    vacuum.enablePresence(timestep)
    print(f"[{role_str}]: VacuumGripper connected and ready!")
else:
    print(f"[{role_str} WARNING]: 'vacuum gripper' not found in toolSlot.")

def grip_on():
    if vacuum:
        vacuum.turnOn()

def grip_off():
    if vacuum:
        vacuum.turnOff()

grip_off()

# Universal Postures:
# Default resting posture (as usual, arm folded alongside pedestal):
DEFAULT_PARK_POSTURE = [-1.571, -1.189, 1.805, 0.955, 1.571, 0.0]
# Board-facing crane posture (Pan +1.571 turns arm to face directly into the chessboard):
FACE_BOARD_POSTURE   = [1.571, -1.189, 1.805, 0.955, 1.571, 0.0]

SIDE_POSTURE = DEFAULT_PARK_POSTURE
READY_POSTURE = FACE_BOARD_POSTURE
HOME_POSTURE = DEFAULT_PARK_POSTURE
UP_POSTURE = FACE_BOARD_POSTURE

# Initial motor position at spawn:
set_arm_angles(DEFAULT_PARK_POSTURE)


# ==============================================================================
# 4. INVERSE KINEMATICS & BASE TRANSFORM SETUP
# ==============================================================================
urdf_name = f"ur10e_skeleton_{'w' if is_white_robot else 'b'}.urdf"
with open(urdf_name, "w") as f:
    f.write(robot.getUrdf())

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    arm_chain = ikpy.chain.Chain.from_urdf_file(urdf_name)

arm_chain.active_links_mask = [
    link.name in set(arm_motor_names) for link in arm_chain.links
]

# Around lines 162-165:
# Prevent flat pose (keeps elbow bent upward in clean crane mode)
arm_chain.links[2].bounds = (-2.8, 0.0)
arm_chain.links[3].bounds = (0.1, 2.8)
# Enforce exact base position and rotation
self_node = robot.getSelf()
if self_node:
    if is_black_robot:
        self_node.getField("translation").setSFVec3f([-0.78, -0.61, 0.22])
        self_node.getField("rotation").setSFRotation([0, 0, 1, 0.0])
        self_node.resetPhysics()
    else:
        self_node.getField("translation").setSFVec3f([-0.68, 0.62, 0.24])
        self_node.getField("rotation").setSFRotation([0, 0, 1, -3.14159])
        self_node.resetPhysics()

    ROBOT_BASE_WORLD = np.array(self_node.getPosition())
    rot_flat = self_node.getOrientation()
    ROT_MATRIX = np.array(rot_flat).reshape((3, 3))
else:
    if is_white_robot:
        ROBOT_BASE_WORLD = np.array([-0.68, 0.62, 0.24])
        theta_w = -np.pi
        c_w, s_w = np.cos(theta_w), np.sin(theta_w)
        ROT_MATRIX = np.array([[c_w, -s_w, 0], [s_w, c_w, 0], [0, 0, 1]])
    else:
        ROBOT_BASE_WORLD = np.array([-0.78, -0.61, 0.22])
        ROT_MATRIX = np.eye(3)

print(f"[{role_str} ONLINE]: Base={ROBOT_BASE_WORLD.round(3)}, Assigned Ranks={sorted(list(my_ranks))}")

def solve_waypoint(world_xy, z_height, initial_guess):
    """Calculates IK angles transformed for robot base rotation, with suction cup pointing straight down."""
    target_world = np.array([world_xy[0], world_xy[1], z_height])
    diff_world = target_world - ROBOT_BASE_WORLD
    target_local = ROT_MATRIX.T @ diff_world
    target_ori = ROT_MATRIX.T @ np.array([0.0, 0.0, -1.0])
    
    return arm_chain.inverse_kinematics(
        target_local,
        target_orientation=target_ori,
        orientation_mode='Y',
        initial_position=initial_guess
    )


# ==============================================================================
# 5. DUAL-ROBOT TURN COORDINATION
# ==============================================================================
TURN_FILE = os.path.join(os.path.dirname(__file__), "shared_turn.txt")

def read_turn():
    try:
        if os.path.exists(TURN_FILE):
            with open(TURN_FILE, "r") as f:
                content = f.read().strip()
                if content.isdigit():
                    return int(content)
    except Exception:
        pass
    return 0

def write_turn(idx):
    try:
        with open(TURN_FILE, "w") as f:
            f.write(str(idx))
    except Exception:
        pass


# ==============================================================================
# 6. MAIN SIMULATION LOOP (Dynamic Dual-Robot Pick-and-Place State Machine)
# ==============================================================================
stage = "STARTUP_LIFT"
state_start_time = 0.0

flight_to_face = 5.5
flight_to_pick = 3.5
flight_travel_to_clear = 2.0
flight_clear_to_touch = 2.0
flight_to_park = 5.5
transfer_duration = 5.0

src_travel_angles = None
src_clear_angles = None
src_touch_angles = None
dst_travel_angles = None
dst_clear_angles = None
dst_touch_angles = None

while robot.step(timestep) != -1:
    now = robot.getTime()

    # --- Phase 1: Initialize game queue and settle into default park posture ---
    if stage == "STARTUP_LIFT":
        if is_black_robot and now < 0.2:
            write_turn(0)

        if now >= 2.0:
            set_arm_angles(DEFAULT_PARK_POSTURE)
            stage = "STARTUP_PAN"
            state_start_time = now

    # --- Phase 2: Settle into default park posture ---
    elif stage == "STARTUP_PAN" and (now - state_start_time) >= 2.5:
        print(f"[{role_str}]: Safely parked at default posture!")
        stage = "IDLE"

    # --- IDLE: Check shared turn queue ---
    elif stage == "IDLE":
        turn_idx = read_turn()
        if turn_idx < len(move_queue):
            src_str, dst_str = move_queue[turn_idx]
            if is_my_move(src_str):
                # My turn!
                current_move_idx = turn_idx
                current_move = (src_str, dst_str)
                print(f"\n[{role_str}] >>> Executing Move #{turn_idx + 1}: {src_str.upper()} -> {dst_str.upper()} <<<")

                src_xy = square_to_world(src_str)
                dst_xy = square_to_world(dst_str)

                ik_guess = [0.0] * len(arm_chain.links)
                ik_guess[1:7] = FACE_BOARD_POSTURE

                # Get the exact piece height:
                piece_z = get_piece_touch_z(src_str)
                src_touch_z = piece_z
                dst_place_z = piece_z   # Holds the exact same piece, so placing height is identical!

                # Solve all waypoints with crane orientation (tool pointing straight down):
                src_touch_angles  = solve_waypoint(src_xy, src_touch_z, ik_guess)
                src_clear_angles  = solve_waypoint(src_xy, Z_CLEAR, src_touch_angles)     # 0.38m vertical
                src_travel_angles = solve_waypoint(src_xy, Z_TRAVEL, src_clear_angles)    # 0.50m cruise

                dst_travel_angles = solve_waypoint(dst_xy, Z_TRAVEL, ik_guess)             # 0.50m destination
                dst_clear_angles  = solve_waypoint(dst_xy, Z_CLEAR, dst_travel_angles)    # 0.38m staging
                dst_touch_angles  = solve_waypoint(dst_xy, dst_place_z, dst_clear_angles) # touchdown flat on board
                
                flight_travel_to_clear = calc_duration(src_travel_angles, src_clear_angles)
                flight_clear_to_touch  = calc_duration(src_clear_angles, src_touch_angles)

                # Step 0a: Move from default posture to face the board
                flight_to_face = calc_duration(DEFAULT_PARK_POSTURE, FACE_BOARD_POSTURE)
                print(f"[{role_str}] Moving to face the board ({flight_to_face:.1f}s)...")
                grip_off()
                set_arm_angles(FACE_BOARD_POSTURE)
                stage = "SWING_TO_BOARD"
                state_start_time = now

    # Step 0b: Arrived facing board; hold here for 0.5s before reaching for piece
    elif stage == "SWING_TO_BOARD" and (now - state_start_time) >= flight_to_face:
        print(f"[{role_str}] Facing board. Pausing for 0.5s before reaching for piece...")
        stage = "PAUSE_FACING_BOARD"
        state_start_time = now

    # Step 0c: After 0.5s pause, fly up to overhead piece at 0.50m cruise height
    elif stage == "PAUSE_FACING_BOARD" and (now - state_start_time) >= 0.5:
        flight_to_pick = calc_duration(FACE_BOARD_POSTURE, src_travel_angles)
        print(f"[{role_str}] Launching to {src_str.upper()} at {Z_TRAVEL}m ({flight_to_pick:.1f}s)...")
        set_arm_angles(src_travel_angles)
        stage = "APPROACH_CRUISE"
        state_start_time = now

    # Step 1a: Arrived directly above piece at 0.50m; lower vertically like an elevator to 0.38m
    elif stage == "APPROACH_CRUISE" and (now - state_start_time) >= flight_to_pick:
        print(f"[{role_str}] Overhead at {src_str.upper()}. Lowering to {Z_CLEAR}m...")
        set_arm_angles(src_clear_angles)
        stage = "APPROACH_CLEAR"
        state_start_time = now

    # 1. Lower onto piece head with SUCTION OFF
    elif stage == "APPROACH_CLEAR" and (now - state_start_time) >= flight_travel_to_clear:
        grip_off()                              # Keep suction OFF while descending!
        set_arm_angles(src_touch_angles)
        stage = "TOUCH_DESCEND"
        state_start_time = now

    # 2. Suction cup has landed flat and flush. NOW turn suction ON!
    elif stage == "TOUCH_DESCEND" and (now - state_start_time) >= flight_clear_to_touch:
        grip_on()                               # Turn suction ON only when seated flat!
        stage = "TOUCH_PICK"
        state_start_time = now

    # Step 2: Confirm vacuum presence and seal
    elif stage == "TOUCH_PICK":
        has_piece = vacuum.getPresence() if vacuum else False
        if has_piece or (now - state_start_time) >= 1.5:
            print(f"[{role_str}] Vacuum seal locked. Presence: {has_piece}")
            stage = "SUCTION"
            state_start_time = now

    # Step 3a: Lift straight up vertically like an elevator to 0.38m (clears all pawn heads!)
    elif stage == "SUCTION" and (now - state_start_time) >= 0.5:
        print(f"[{role_str}] Lifting vertically to {Z_CLEAR}m...")
        set_arm_angles(src_clear_angles)
        stage = "LIFT_CLEAR"
        state_start_time = now

    # Step 3b: Rise to 0.50m cruise altitude
    elif stage == "LIFT_CLEAR" and (now - state_start_time) >= flight_clear_to_touch:
        print(f"[{role_str}] Rising to {Z_TRAVEL}m cruise altitude...")
        set_arm_angles(src_travel_angles)
        stage = "LIFT_CRUISE"
        state_start_time = now

    # Step 4: Fly high across board maintaining 0.50m (no mid-air sag!)
    elif stage == "LIFT_CRUISE" and (now - state_start_time) >= flight_travel_to_clear:
        transfer_duration = calc_duration(src_travel_angles, dst_travel_angles)
        print(f"[{role_str}] Transferring high at {Z_TRAVEL}m to {dst_str.upper()} ({transfer_duration:.1f}s)...")
        set_arm_angles(dst_travel_angles)
        stage = "TRANSFER"
        state_start_time = now

    # Step 5: Arrived directly overhead at destination at 0.50m -> Drop down to 0.30m
    elif stage == "TRANSFER" and (now - state_start_time) >= transfer_duration:
        flight_clear_duration = calc_duration(dst_travel_angles, dst_clear_angles)
        print(f"[{role_str}] Overhead at {dst_str.upper()}. Dropping to {Z_CLEAR}m ({flight_clear_duration:.1f}s)...")
        set_arm_angles(dst_clear_angles)
        stage = "STAGE_CLEAR"
        state_start_time = now

    # Step 6: Arrived at 0.30m -> Drop down to 0.15m (or piece touch height) to place
    elif stage == "STAGE_CLEAR" and (now - state_start_time) >= flight_clear_duration:
        flight_touch_duration = calc_duration(dst_clear_angles, dst_touch_angles)
        print(f"[{role_str}] Lowering to place on board at {dst_place_z:.3f}m ({flight_touch_duration:.1f}s)...")
        set_arm_angles(dst_touch_angles)
        stage = "TOUCH_PLACE"
        state_start_time = now

    # Step 7: Piece is sitting flat on the board surface: turn vacuum OFF
    elif stage == "TOUCH_PLACE" and (now - state_start_time) >= flight_touch_duration:
        grip_off()                              # Cut suction!
        print(f"[{role_str}] Placed piece. Releasing vacuum suction...")
        stage = "RELEASE"
        state_start_time = now

    # Step 8: Wait 0.5s for air pressure to release, then retract vertically back to 0.30m
    elif stage == "RELEASE" and (now - state_start_time) >= 0.5:
        flight_retract_duration = calc_duration(dst_touch_angles, dst_clear_angles)
        print(f"[{role_str}] Retracting vertically back to {Z_CLEAR}m...")
        set_arm_angles(dst_clear_angles)        # Retract vertically
        stage = "RETRACT"
        state_start_time = now

    # Step 9: Return smoothly to default parked posture
    elif stage == "RETRACT" and (now - state_start_time) >= flight_retract_duration:
        flight_to_park = calc_duration(dst_clear_angles, DEFAULT_PARK_POSTURE)
        print(f"[{role_str}] Returning to default parked posture ({flight_to_park:.1f}s)...")
        set_arm_angles(DEFAULT_PARK_POSTURE)
        stage = "RETURN"
        state_start_time = now