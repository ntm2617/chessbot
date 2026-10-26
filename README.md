---

# 🤖 Dual-Robot Collaborative Chess Simulation in Webots

An autonomous multi-robot chess simulation built with **Cyberbotics Webots**, featuring two **Universal Robots UR10e** robotic arms equipped with vacuum grippers. The robots autonomously take turns executing chess moves, handling piece captures (eliminations), and navigating complex 3D clearance trajectories above the chessboard without collisions.

---

## 🌟 Key Features

* **Dual-Robot Turn Coordination**: File-based synchronization (`shared_turn.txt`) guarantees turn-based mutual exclusion so both robots never move simultaneously or collide in mid-air.
* **Autonomous Piece Capture (Elimination)**: Supports capture moves. When a piece is taken, the active robot picks up the captured piece, carries it off the board into an organized table graveyard, and then executes its move.
* **Synchronized Joint Trajectories (Anti-Sag)**: Dynamically scales individual motor velocities so all 6 joints finish their motion simultaneously, eliminating the mid-air arm sag that causes mid-flight collisions.
* **Hierarchical 2-Step Elevator Motion**:
  * **Z = 0.50 m (Cruise)**: High-altitude flight safely above all pieces.
  * **Z = 0.30 m (Staging)**: Vertical elevator alignment directly over target squares.
  * **Z = 0.15 m - 0.18 m (Touchdown)**: Calibrated surface heights for pawns, knights, bishops, rooks, queens, and kings.
* **180° Backward Park Posture**: Arms fold safely backward away from the chessboard when idle, preventing physical interference.
* **4-Point Rigid Vacuum Gripper**: Uses a 4-point planar contact model in Webots to eliminate piece dangling or tilting during flight.

---

## 🛠️ System Architecture & Flowchart

The following state machine governs the behavior of both robots:

<img width="1044" height="3405" alt="mermaid-diagram" src="https://github.com/user-attachments/assets/6baaec4e-1ced-4d9d-8ab0-1e15dd17ba8f" />

---

## 📂 Project Directory Structure

```text
my_first_simulation/
├── controllers/
│   └── ur10e_chess/
│       ├── ur10e_chess.py         # Main autonomous Python robot controller
│       ├── shared_turn.txt        # Inter-process turn synchronization file
│       ├── ur10e_skeleton_b.urdf  # Exported URDF kinematic model for Black Robot
│       └── ur10e_skeleton_w.urdf  # Exported URDF kinematic model for White Robot
├── worlds/
│   ├── chessbot.wbt               # Main Webots 3D simulation world
│   ├── 3d_models/                 # High-resolution STL/STEP models of chess pieces
│   │   ├── pawn.stl, knight.stl, bishop.stl, rook.stl, queen.stl, king.stl
│   └── textures/                  # Textures for chessboard and table
└── README.md
```


---

## 📦 Requirements & Installation

1. **Cyberbotics Webots**: Version R2025a or later ([Download Webots](https://cyberbotics.com/)).
2. **Python**:
3. **Required Python Packages**:
   Install the kinematic and mathematical libraries:
   ```bash
   pip install ikpy numpy
   ```

---

## 🚀 How to Run the Simulation

1. Launch **Webots**.
2. Go to **File -> Open World...** and select:
   ```text
   worlds/chessbot.wbt
   ```
3. Press the **Play (Run)** button at the top of the Webots interface.
4. The robots will initialize, swing into their safe parking postures, and begin executing moves from `move_queue` in turn order.

---

## ⚙️ Move Queue Configuration

Moves are scheduled in `controllers/ur10e_chess/ur10e_chess.py` inside the `move_queue` list:

```python
move_queue = [
    # Normal move: (source, destination)
    ("e2", "e4"),
    
    # Capture move: (source, destination, captured_square)
    ("h8", "a1", "a1"),  # Eliminates white piece on A1 first, then moves H8 to A1!
]
```

* **Standard Moves** take 2 strings: `("src", "dst")`.
* **Capture Moves** take 3 strings: `("src", "dst", "elim")`. The robot will automatically clear the piece at `"elim"` to the off-board graveyard before carrying out the move.

---

## 📐 Piece Touchdown Calibrations

| Piece Type | Touchdown Height ($Z$) |
| :--- | :--- |
| **Pawns** | `0.145m` |
| **Bishops / Rooks / Queen** | `0.165m` |
| **Knights** | `0.173m` |
| **Kings** | `0.178m` |

## 🍿 Simulation



https://github.com/user-attachments/assets/4bb16b9c-3135-48fa-9ae5-9bd04799ca3c

