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
    ("h1", "a5"),   # Black test move
    # ("e2", "e4"), # White move
    # ("e7", "e5"), # Black move
]

# Height settings:
Z_TRAVEL = 0.50        # High cruise height (maintains 0.5m with elbow up across the board)
Z_CLEAR = 0.30         # Overhead clearance & staging height (0.3m above board)
Z_TOUCH = 0.15         # Touchdown contact height to grab / place


# ==============================================================================
# 2. CHESSBOARD 2-POINT GROUND TRUTH CALIBRATION
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
    diffs = [abs(a - b) for a, b in zip(angles_a[1:7], angles_b[1:7])]
    return (max(diffs) / v) + 0.8

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

# Universal Crane Posture (Pan +1.571 points directly into the board in local frame)
UP_POSTURE = [1.571, -1.189, 1.805, 0.955, 1.571, 0.0]
READY_POSTURE = [1.571, -1.189, 1.805, 0.955, 1.571, 0.0]
SIDE_POSTURE = [0.0, -1.189, 1.805, 0.955, 1.571, 0.0]

# Initial motor position at spawn:
for motor, pos in zip(arm_motors, SIDE_POSTURE):
    motor.setPosition(pos)


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

# Prevent flat pose (keeps elbow bent upward in crane mode)
# arm_chain.links[2].bounds = (-2.8, 0.0)
# arm_chain.links[3].bounds = (0.1, 2.8)
# Prevent flat pose (keeps elbow bent upward in clean crane mode)
arm_chain.links[2].bounds = (-2.8, 0.0)
arm_chain.links[3].bounds = (0.1, 2.8)
# Read EXACT world position and 3x3 orientation directly from Webots Supervisor node
self_node = robot.getSelf()
if self_node:
    ROBOT_BASE_WORLD = np.array(self_node.getPosition())
    rot_flat = self_node.getOrientation()
    ROT_MATRIX = np.array(rot_flat).reshape((3, 3))
else:
    if is_white_robot:
        ROBOT_BASE_WORLD = np.array([-0.68, 0.87, 0.24])
        theta_w = -np.pi
        c_w, s_w = np.cos(theta_w), np.sin(theta_w)
        ROT_MATRIX = np.array([[c_w, -s_w, 0], [s_w, c_w, 0], [0, 0, 1]])
    else:
        ROBOT_BASE_WORLD = np.array([-0.78, -0.56, 0.14])
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

def set_arm_angles(angles):
    for motor, angle in zip(arm_motors, angles[1:7]):
        motor.setPosition(angle)


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
WAYPOINT_TIME = 3.5
flight_to_pick = 3.5

src_travel_angles = None
src_clear_angles = None
src_touch_angles = None
dst_travel_angles = None
dst_clear_angles = None
dst_touch_angles = None
transfer_duration = 5.0

while robot.step(timestep) != -1:
    now = robot.getTime()

    # --- Phase 1: Initialize game queue and settle into side posture ---
    if stage == "STARTUP_LIFT":
        if is_black_robot and now < 0.2:
            write_turn(0)

        if now >= 2.0:
            for motor, pos in zip(arm_motors, SIDE_POSTURE):
                motor.setPosition(pos)
            stage = "STARTUP_PAN"
            state_start_time = now

    # --- Phase 2: Settle into side posture ---
    elif stage == "STARTUP_PAN" and (now - state_start_time) >= 2.5:
        print(f"[{role_str}]: Safely parked at side posture!")
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
                ik_guess[1:7] = READY_POSTURE

                # Knights, Bishops, Rooks, Queen, King are 0.175m tall. Pawns are 0.145m tall.
                src_rank = int(src_str.strip()[-1])
                src_touch_z = 0.145 if src_rank in {2, 7} else 0.175
                dst_place_z = src_touch_z

                # Solve trajectory waypoints:
                src_touch_angles  = solve_waypoint(src_xy, src_touch_z, ik_guess)
                src_clear_angles  = solve_waypoint(src_xy, Z_CLEAR, src_touch_angles)     # 0.30m vertical lift
                src_travel_angles = solve_waypoint(src_xy, Z_TRAVEL, src_clear_angles)    # 0.50m cruise height

                dst_travel_angles = solve_waypoint(dst_xy, Z_TRAVEL, ik_guess)             # 0.50m overhead destination
                dst_clear_angles  = solve_waypoint(dst_xy, Z_CLEAR, dst_travel_angles)    # 0.30m staging
                dst_touch_angles  = solve_waypoint(dst_xy, dst_place_z, dst_clear_angles) # 0.0° tilt touchdown

                # Move directly from parked base position (-0.78, -0.56, 0.14) to piece at 0.50m:
                flight_to_pick = calc_duration(SIDE_POSTURE, src_travel_angles)
                print(f"[{role_str}] Moving from base position (-0.78, -0.56, 0.14) to {src_str.upper()} at {Z_TRAVEL}m ({flight_to_pick:.1f}s)...")
                grip_off()
                set_arm_angles(src_travel_angles)
                stage = "APPROACH_CRUISE"
                state_start_time = now

    # Step 1a: Arrived directly above piece at 0.50m; lower vertically to 0.30m
    elif stage == "APPROACH_CRUISE" and (now - state_start_time) >= flight_to_pick:
        print(f"[{role_str}] Overhead at {src_str.upper()}. Lowering to {Z_CLEAR}m...")
        set_arm_angles(src_clear_angles)
        stage = "APPROACH_CLEAR"
        state_start_time = now

    # Step 1b: Drop straight down to piece head to pick
    elif stage == "APPROACH_CLEAR" and (now - state_start_time) >= 1.5:
        print(f"[{role_str}] Lowering to {src_touch_z}m to grab...")
        grip_on()
        set_arm_angles(src_touch_angles)
        stage = "TOUCH_PICK"
        state_start_time = now

    # Step 2: Confirm vacuum presence
    elif stage == "TOUCH_PICK":
        has_piece = vacuum.getPresence() if vacuum else False
        if has_piece or (now - state_start_time) >= 3.0:
            print(f"[{role_str}] Touchdown complete. Vacuum presence: {has_piece}")
            stage = "SUCTION"
            state_start_time = now

    # Step 3a: Lift straight up vertically like an elevator to 0.30m
    elif stage == "SUCTION" and (now - state_start_time) >= 1.0:
        print(f"[{role_str}] Lifting vertically to {Z_CLEAR}m...")
        set_arm_angles(src_clear_angles)
        stage = "LIFT_CLEAR"
        state_start_time = now

    # Step 3b: Rise to 0.50m cruise altitude
    elif stage == "LIFT_CLEAR" and (now - state_start_time) >= 1.5:
        print(f"[{role_str}] Rising to {Z_TRAVEL}m cruise altitude...")
        set_arm_angles(src_travel_angles)
        stage = "LIFT_CRUISE"
        state_start_time = now

    # Step 4: Transfer across the board maintaining 0.50m height
    elif stage == "LIFT_CRUISE" and (now - state_start_time) >= 2.0:
        transfer_duration = calc_duration(src_travel_angles, dst_travel_angles)
        print(f"[{role_str}] Transferring at {Z_TRAVEL}m to {dst_str.upper()} ({transfer_duration:.1f}s)...")
        set_arm_angles(dst_travel_angles)
        stage = "TRANSFER"
        state_start_time = now

    # Step 5a: Arrived overhead at destination at 0.50m; lower to 0.30m first
    elif stage == "TRANSFER" and (now - state_start_time) >= transfer_duration:
        print(f"[{role_str}] Overhead at {dst_str.upper()}. Lowering to {Z_CLEAR}m first...")
        set_arm_angles(dst_clear_angles)
        stage = "STAGE_CLEAR"
        state_start_time = now

    # Step 5b: Lower gently to board surface with 0.0° tilt
    elif stage == "STAGE_CLEAR" and (now - state_start_time) >= 1.5:
        print(f"[{role_str}] Placing piece gently at {dst_place_z}m...")
        set_arm_angles(dst_touch_angles)
        stage = "TOUCH_PLACE"
        state_start_time = now

    # Step 6: Release suction
    elif stage == "TOUCH_PLACE" and (now - state_start_time) >= 2.5:
        grip_off()
        print(f"[{role_str}] [-] Vacuum OFF: Piece placed cleanly with 0.0° tilt.")
        stage = "RELEASE"
        state_start_time = now

    # Step 7: Retract straight back up to 0.30m
    elif stage == "RELEASE" and (now - state_start_time) >= 0.5:
        print(f"[{role_str}] Retracting straight up to {Z_CLEAR}m...")
        set_arm_angles(dst_clear_angles)
        stage = "RETRACT"
        state_start_time = now

    # Step 8: Return smoothly to parked side posture
    elif stage == "RETRACT" and (now - state_start_time) >= 1.5:
        for motor, pos in zip(arm_motors, SIDE_POSTURE):
            motor.setPosition(pos)
        stage = "RETURN"
        state_start_time = now

    # Step 9: Complete move and advance turn
    elif stage == "RETURN" and (now - state_start_time) >= 3.0:
        print(f"[{role_str}] [✓] Move #{current_move_idx + 1} {src_str.upper()} -> {dst_str.upper()} Complete!\n")
        write_turn(current_move_idx + 1)
        current_move = None
        current_move_idx = None
        stage = "IDLE"