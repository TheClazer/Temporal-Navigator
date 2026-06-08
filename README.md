# Temporal Navigator — Pathfinding under Dynamic Hazards

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Language](https://img.shields.io/badge/language-C-blue)
![Library](https://img.shields.io/badge/library-Raylib%205.0-red)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey)
![Course](https://img.shields.io/badge/DSA%20%2B%20DAA-Design%20%26%20Analysis-purple)

**Temporal Navigator** is an interactive pathfinding simulation in C + Raylib. An agent must escape a grid as hazards (fire/gas) spread through it in time, replanning on the fly to stay ahead of the danger front.

It is built as a **Design & Analysis of Algorithms (DAA)** artifact as much as a Data-Structures one: alongside the live data-structure visualizations (queue, linked list) and the min-heap priority queue driving the search, it instruments the algorithms while they run — counting nodes expanded, edges relaxed, heap operations and peak frontier — and lets you run **Dijkstra vs A\* head-to-head on the same map** to *measure* the difference between an uninformed and an informed search.

> See **[`ANALYSIS.md`](ANALYSIS.md)** for the full design-and-analysis writeup: complexity derivations, A\* admissibility/optimality proof for this grid, the time-expanded state-space model, and viva talking points.

---

## 🎯 Why this is a Design & Analysis project

| DAA theme | Where it shows up |
| :--- | :--- |
| **Algorithm design paradigms** | BFS (level-order traversal), Dijkstra & A\* (greedy best-first + priority queue), the safety-buffer planner (constrained shortest path), replanning (online re-optimization). |
| **Informed vs uninformed search** | Press **`A`** to switch the live planner between **Dijkstra** and **A\*** (Dijkstra + Manhattan heuristic). |
| **Empirical complexity analysis** | A live **Complexity Instrumentation** panel: nodes expanded, edges relaxed, heap pushes, peak frontier (space), path cost/length — the measured analogues of the Big-O work terms. |
| **Optimality & admissibility** | Press **`C`** to run **both** planners on the current map. The tool proves they return the **same optimal cost** while A\* expands **far fewer nodes**, and visualizes both explored sets on the grid. |
| **Worst/best/average case** | The comparison percentage literally measures where the current map sits on the spectrum (open maps favor A\*; a single forced corridor erases its edge). |

---

## 🚀 Features

- **Dual planners — Dijkstra & A\***: a single unified solver runs as plain Dijkstra (`f = g`) or A\* (`f = g + h`, Manhattan heuristic). Dijkstra behavior is byte-identical to the original; A\* is provably optimal on this grid.
- **Live complexity instrumentation**: every plan reports nodes expanded, edges relaxed, heap pushes, peak frontier size (space), path cost and length — with the theoretical Big-O shown right beside the measured numbers.
- **Comparison mode (`C`)**: runs both planners on the frozen current state, asserts identical optimal cost (the admissibility check), reports A\*'s node-expansion savings, and tints the two explored sets so you can *see* A\*'s tighter search cone inside Dijkstra's flood.
- **Dynamic temporal pathfinding**: Dijkstra/A\* with a time-based **safety buffer** — an edge is feasible only if `start_time + g + SAFETY_BUFFER < hazard_arrival_time`.
- **Online replanning**: a scripted "Wall Collapse" event invalidates the plan mid-run; the agent re-optimizes from its current cell. A least-risky **fallback** path is used when no fully safe path exists.
- **Pre-flight reachability ("Waterflow")**: a BFS flood-fill that confirms the goal is reachable before planning.
- **Interactive map editor**: draw walls, place hazard sources, move start/end.
- **Data-structure visualizations**: live **Queue** (BFS frontier) and **Linked List** (the agent's planned path) in the analysis bar.

---

## 🎮 Controls

### Algorithm / Analysis  (the DAA additions)
| Key | Action |
| :--- | :--- |
| **`A`** | **Toggle the active planner: Dijkstra ⇄ A\*** (re-plans immediately in simulation) |
| **`C`** | **Comparison mode** — run both planners, show the head-to-head card + explored-set overlay (toggle off with `C`) |

### Tools & Editing  (Setup)
| Key | Action |
| :--- | :--- |
| **`1`** | Wall Tool (place obstacles) |
| **`2`** | Hazard Tool (place hazard sources) |
| **`3`** | Start Point |
| **`4`** | End Point |
| **`5`** | Eraser |
| **`L-Click`** | Place selected element |
| **`R-Click`** | Quick delete (wall/hazard) |

### Simulation State
| Key | Action |
| :--- | :--- |
| **`ENTER`** | Commit map: run hazard pre-calc & Waterflow reachability check |
| **`SPACE`** | Start simulation (enabled once the reachability flood-fill completes) |
| **`R`** | Hard reset (restore default map) |
| **`BACKSPACE`** | Soft reset (keep current map layout) |

---

## 🧠 Algorithms & Complexity

`V = R·C = 2500` cells (vertices), `E = 9800 ≈ 4V` directed edges (4-neighbour grid). Edges are **unit cost**, so the accumulated cost `g(n)` equals the number of steps from the source.

| Algorithm | Role | Time | Space |
| :--- | :--- | :--- | :--- |
| **BFS** | Hazard propagation (multi-source) + Waterflow reachability | `O(V + E)` | `O(V)` |
| **Dijkstra** (binary min-heap) | Core planner; greedy best-first by `g(n)` | `O((V + E) log V)` | `O(V)` |
| **A\*** (Dijkstra + Manhattan `h`) | Informed planner; orders by `f = g + h` | `O((V + E) log V)` worst case; far fewer expansions in practice | `O(V)` |
| **MinHeap** | extract-min / decrease-key / peek | `O(log V)` / `O(log V)` / `O(1)` | `O(V)` |

A\* uses `h(n) = |dx| + |dy|` (Manhattan distance), which is **admissible and consistent** on a 4-connected unit-cost grid, so A\* returns the **same optimal cost** as Dijkstra. Full derivations and the optimality proof are in **[`ANALYSIS.md`](ANALYSIS.md)**.

---

## 🛠️ Build & Run

### Prerequisites
- Windows OS (10 or 11)

### Setup & Run
**No manual installation of compilers or libraries is required.** The build script handles everything.

1. **Double-click `build.bat`** (or run it from a terminal).
2. The script will automatically:
   - Download & set up the portable compiler (w64devkit) if missing.
   - Download & set up Raylib 5.0 if missing.
   - Compile the simulation.
   - Launch the application.

> The first run needs an internet connection to fetch dependencies (~150 MB). Subsequent runs are offline and instant.

### Quick demo flow
1. Launch → **Setup**. Draw walls/hazards or keep the default map.
2. Press **`ENTER`** → Waterflow reachability check → **`SPACE`** to start.
3. Watch the agent plan and replan around the spreading fire (note the **Wall Collapse** event).
4. Press **`A`** to switch Dijkstra ⇄ A\* and watch the live metrics change.
5. Press **`C`** for the **head-to-head**: same optimal cost, fewer A\* expansions, with both explored sets overlaid on the grid.

---

## 📂 Project Structure

- `raylib_viz.c` — main GUI simulation (rendering, state machine, planners, instrumentation, comparison mode).
- `hazard_pathfinding.c` — headless console version (ANSI rendering + `metrics.csv` logging) for benchmarking.
- `ANALYSIS.md` — the Design & Analysis writeup (complexity, optimality, paradigms, viva notes).
- `build.bat` — Windows auto-build script.
- `raylib/` — Raylib headers & libraries (downloaded by the build script).

---
*Created for the Advanced Agentic Coding Project · extended for Design & Analysis of Algorithms.*
