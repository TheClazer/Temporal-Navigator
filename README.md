# Temporal Disaster Navigator

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Language](https://img.shields.io/badge/language-C-blue)
![Library](https://img.shields.io/badge/library-Raylib%205.0-red)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey)

**Temporal Disaster Navigator** is an interactive pathfinding simulation built in C using Raylib. It demonstrates advanced pathfinding algorithms in a dynamic, hazardous environment. The agent must navigate a grid populated with growing hazards (e.g., fire, gas), making real-time decisions to balance path efficiency against safety risks.

## 🚀 Features

- **Dynamic Pathfinding**: Uses Dijkstra's algorithm with dynamic edge weights to navigate around spreading hazards.
- **Safety & Risk Management**:
  - **Safety Buffer**: The agent attempts to maintain a time-based safety buffer from oncoming hazards.
  - **Fallback Logic**: If no safe path exists, the agent intelligently selects the "least risky" path rather than giving up.
- **Interactive Map Editor**: Fully customizable grid. Draw walls, place hazard sources, and move start/end goals.
- **Pre-Flight Reachability Check**: Includes a "Waterflow" (Flood Fill / BFS) visualizer to verify map solvability before the simulation begins.
- **DSA Visualization**: Real-time rendering of internal data structures:
  - **Queue Visualization** for the Flood Fill process.
  - **Linked List Visualization** for the agent's planned path.

## 🎮 Controls

### Tools & Editing
| Key | Action |
| :--- | :--- |
| **`1`** | **Wall Tool** (Place obstacles) |
| **`2`** | **Hazard Tool** (Place hazard sources) |
| **`3`** | **Start Point** (Set agent spawn) |
| **`4`** | **End Point** (Set target destination) |
| **`5`** | **Eraser** (Remove obstacles/hazards) |
| **`L-Click`** | Place selected element |
| **`R-Click`** | Quick delete (Wall/Hazard) |

### Simulation State
| Key | Action |
| :--- | :--- |
| **`ENTER`** | **Commit Map**: Runs Hazard Pre-calc & Starts Waterflow Check |
| **`SPACE`** | **Start Simulation**: Begins agent movement (after check passes) |
| **`R`** | **Hard Reset**: Restores default map and settings |
| **`BACKSPACE`**| **Soft Reset**: Resets simulation but keeps current map layout |

## 🛠️ Build & Run

### Prerequisites
- Windows OS (10 or 11)

### Setup & Run
**No manual installation of compilers or libraries is required!** The build script handles everything automatically.

1. **Double-click `build.bat`** (or run it from a terminal).
2. The script will automatically:
   - Download and setup the portable compiler (w64devkit) if missing.
   - Download and setup the Raylib library if missing.
   - Compile the simulation.
   - Launch the application.

> **Note:** The first run requires an internet connection to download dependencies (~150MB). Subsequent runs will be offline and instant.

## 🧠 Algorithms Used

1. **Breadth-First Search (BFS)**:
   - Used for the **Hazard Propagation** logic (calculating arrival times for every cell).
   - Used for the **Waterflow** pre-flight check to determine reachability.

2. **Dijkstra's Algorithm**:
   - The core pathfinding engine.
   - **Cost Function**: `Base Cost + Hazard Penalty`.
   - Dynamic updates allow the agent to replan if the environment changes (e.g., "Wall Collapse" events).

## 📂 Project Structure

- `raylib_viz.c`: Main source code containing the simulation loop, rendering, and logic.
- `build.bat`: Windows batch file for compilation.
- `raylib/`: Directory containing Raylib headers and libraries.

---
*Created for the Advanced Agentic Coding Project.*
