# Design & Analysis of Algorithms — Temporal Navigator

This document is the algorithmic companion to the code. It explains **what** each algorithm is, **why** it was chosen (design), and **how it behaves** (analysis): asymptotic complexity, optimality conditions, and the empirical experiment the tool lets you run live.

**Notation.** The grid is `R × C = 50 × 50`, so `V = R·C = 2500` vertices (cells). It is 4-connected, giving `E = 2·(R·(C−1) + C·(R−1)) = 9800` directed edges (`≈ 4V`; border cells have degree 2–3, so `E < 4V` strictly). Every edge has **unit cost**, so in Dijkstra/A\* the accumulated cost `g(n)` is exactly the number of steps from the source.

---

## 1. The problem, precisely

An agent starts at `start` and must reach `end`. Hazards spread outward from source cells; cell `c` becomes lethal at time `hazard_arrival_time(c)`. The agent moves one cell per time unit. A move into cell `c'` arriving at time `t` is **feasible** only if it keeps a safety margin:

```
t + SAFETY_BUFFER  <  hazard_arrival_time(c')      (strict-safe planning)
```

This is a **constrained shortest-path problem on a time-expanded graph**: the true state is `(cell, time)`, and an edge `(c, t) → (c', t+1)` exists only if `c'` is still safe at `t+1` (with buffer). Because every move advances time by exactly one unit *and* costs exactly one unit, a cell's arrival time equals its `g`-cost — which collapses the time dimension and lets us solve the problem on the 2-D grid with a single deadline test per edge, instead of materializing the full `O(V·T)` time-expanded graph. **Replanning** handles the cases where the constraint set itself changes mid-flight (the scripted *Wall Collapse*): the time-expanded graph is revealed online rather than known in advance.

---

## 2. Algorithms & design paradigms

| Technique | Paradigm | Used for |
| :--- | :--- | :--- |
| **BFS** | Level-order graph traversal | Hazard propagation (multi-source) + Waterflow reachability |
| **Dijkstra** | Greedy best-first + priority queue | Core planner, ordered by `g(n)` |
| **A\*** | Greedy best-first + admissible heuristic | Informed planner, ordered by `f(n) = g(n) + h(n)` |
| **Safety-buffer planner** | Constrained shortest path | Feasible-subgraph search under the temporal deadline |
| **Replanning** | Incremental / online re-optimization | Re-solve from the current cell when the plan becomes infeasible |

### 2.1 Breadth-First Search
Two uses, both on an *unweighted* notion of distance, so a plain FIFO queue is optimal and a priority queue would be wasteful:

1. **Hazard propagation** — a **multi-source** BFS seeded from every hazard cell assigns each cell its `hazard_arrival_time` (one BFS ring = one time step here, scaled by 10). Because every edge advances the frontier by the same amount, BFS yields correct shortest-time-to-ignition layers.
2. **Waterflow reachability** — a single-source BFS flood-fill from `start` that confirms `end` is reachable *before* planning, so the planner is never invoked on an unsolvable map.

**Complexity:** `O(V + E)` time, `O(V)` space.

### 2.2 Dijkstra's algorithm
Repeatedly extract the unexpanded cell of least `g(n)` from a **binary min-heap**, then relax its 4 neighbours. The heap stores one entry per cell and supports `decrease-key` via a `pos[]` index, so there are no stale duplicates.

**Complexity:** `V` extract-mins and up to `E` decrease-keys, each `O(log V)` ⇒ **`O((V + E) log V)`** time, `O(V)` space.

### 2.3 A\* search
A\* is Dijkstra ordered by `f(n) = g(n) + h(n)` instead of `g(n)` alone, where `h(n) = |dx| + |dy|` is the Manhattan distance to `end`. The heuristic biases expansion toward the goal, so A\* expands a **subset** of the cells Dijkstra would.

A subtle but critical implementation point: the heap key is `f`, but **`g` is the truth**. The solver stores pure `g` in `dist[][]` and pushes `f = g + h` only as the heap priority; every feasibility test, relaxation comparison, and the final path cost use `g`, read from `dist[][]` — never the value popped from the heap (which is `f` for A\*). Setting `h ≡ 0` recovers Dijkstra exactly, which is why the two share one code path.

**Complexity:** worst case **`O((V + E) log V)`** — identical to Dijkstra (a zero/weak heuristic degenerates A\* into Dijkstra). On informative maps it expands far fewer nodes; best case approaches `O(path_length · log V)` when the heuristic walks almost straight to the goal.

---

## 3. A\* optimality, admissibility & consistency (for *this* grid)

> A heuristic `h` is **admissible** if it never overestimates the true remaining cost. On this grid the only legal moves are the 4 cardinal directions and every edge costs exactly 1, so reaching the goal from cell `n` needs **at least** `|dx| + |dy|` moves — you cannot close one row and one column of separation in fewer than their sum of unit steps, and diagonal shortcuts do not exist. Therefore `h(n) = |dx| + |dy| ≤ true_cost(n)` always: **Manhattan distance is admissible here**, which guarantees A\* returns an optimal-cost path.

> The Manhattan heuristic is also **consistent (monotone)**: for any cell `n` and neighbour `n'`, `h(n) ≤ cost(n, n') + h(n') = 1 + h(n')`, because a single cardinal step changes `|dx| + |dy|` by exactly ±1. Consistency is stronger than admissibility: each cell is expanded at most once (its `g` is final when first extracted), so A\* needs no costly node re-openings on this grid.

> **Interaction with the temporal constraint.** The safety buffer only *removes* edges from the graph; it never changes the cost of an edge that remains (each stays unit cost). `h` is computed against the unconstrained grid geometry, where it is a true lower bound — and a lower bound on the unconstrained distance is also a lower bound on any longer, *constrained* distance. So `h` stays admissible on the feasible subgraph: Dijkstra and A\* explore the **same feasible subgraph** and return the **same optimal step-cost**; A\* simply expands fewer of those feasible cells. Comparison Mode (`C`) demonstrates exactly this, and asserts the equal-cost property at runtime.

> **A design decision to defend:** keeping edges unit-cost is what keeps *both* the temporal model (1 step = 1 time unit) *and* the Manhattan heuristic sound. If we allowed diagonal moves, Manhattan would *over*-count and break optimality (octile/Chebyshev would be the correct heuristic); if we added weighted terrain, the `arrival_time = g` identity that collapses the time dimension would break.

---

## 4. Empirical vs theoretical (what the instrumentation proves)

The live counters turn an abstract Big-O claim into a measured one:

| Counter | Meaning | Analyses |
| :--- | :--- | :--- |
| **Nodes expanded** | `extractMin` operations on real cells (each marks a cell closed) | empirical analogue of the work term |
| **Edges relaxed** | successful `dist[]` improvements | exposes the `E` term's constant factor |
| **Heap pushes** | insertions into the min-heap | exposes the `V` term's constant factor |
| **Peak frontier** | high-water mark of the heap size | empirical measure of **space**, bounded by `O(V)` |
| **Path cost / length** | `dist[end]` and the linked-list length | the solution being optimized |

**The experiment (`C`).** Running Dijkstra and A\* on the *identical* map holds `V`, `E`, and the optimal cost fixed, isolating the single variable of interest: how many nodes each strategy expands. That is the textbook informed-vs-uninformed search comparison, executed on a map you draw yourself. The reported "% reduction" measures how much exploration the heuristic saved.

---

## 5. Viva / demo talking points

1. **Best / worst / average case.** Worst case for both Dijkstra and A\* is `O((V+E) log V)` — every cell enters and leaves the heap once at `O(log V)` per op. A\*'s best case approaches `O(path_length · log V)` with a perfectly informative heuristic. Average case depends on map topology: open maps favour A\* heavily; a single forced corridor erases its advantage because there is nothing to prune. The Comparison percentage *measures* where the current map sits on that spectrum.

2. **Why a heap beats array-scan Dijkstra (`O((V+E) log V)` vs `O(V²)`).** Naïve Dijkstra scans all `V` cells for the minimum each iteration → `V × V = O(V²) ≈ 6.25M` ops on this grid. The binary min-heap makes extract-min and decrease-key `O(log V)` (~12 for `V=2500`) → `≈ (2500+9800)·12 ≈ 150K` ops — roughly a **40× reduction**. For sparse graphs like a 4-neighbour grid (`E = O(V)`), the heap is the right structure.

3. **Why A\*'s worst case equals Dijkstra's.** A\* *is* Dijkstra ordered by `f = g + h`. Set `h ≡ 0` and `f = g`. Any weak/uninformative heuristic makes A\* expand the same set Dijkstra would, so the heuristic can only help (or, with `h=0`, not hurt) — it never raises the asymptotic worst case. Hence "same worst case, better in practice," not "faster Big-O."

4. **Admissibility is what makes A\* *correct*, not just fast.** Optimality is conditional: A\* returns an optimal path **iff** `h` never overestimates. Manhattan is admissible here precisely because edges are unit cost and moves are 4-directional. Diagonals would break it. This is the one heuristic-design fact to be ready to defend.

5. **The time-expanded state-space view.** Although we plan on a 2-D grid of 2500 cells, the real state is `(cell, time)`. Because every edge advances time by one unit and costs one unit, a cell's arrival time equals its `g`-cost; this collapses the 3-D `(cell, time)` graph to a 2-D search with one deadline test per edge, avoiding the `O(V·T)` blow-up. Replanning handles the online reveal of the constraint set.

6. **Right tool per sub-problem.** BFS (FIFO) for the *unweighted* hazard spread and reachability; min-heap Dijkstra/A\* for the *cost-weighted, goal-directed* plan. Paying for an `O(log V)` priority queue where a FIFO suffices would be strictly wasteful — choosing the cheapest correct structure per sub-problem is itself a design-and-analysis decision.

---

## 6. Data structures (the DSA layer)

| Structure | Implementation | Role | Key ops |
| :--- | :--- | :--- | :--- |
| **Queue (FIFO)** | array + front/rear indices | BFS frontier (hazard spread, Waterflow) | enqueue/dequeue `O(1)` |
| **Binary Min-Heap** | array + `pos[]` index for `decrease-key` | Dijkstra/A\* frontier (priority queue) | extract-min / decrease-key `O(log V)`, peek `O(1)` |
| **Linked List** | singly-linked, prepend on backtrack | the agent's reconstructed path | `pushFront` `O(1)` |
| **2-D grid** | static `Cell[ROWS][COLS]` | world state, costs, hazard times | random access `O(1)` |

---
*Companion to `raylib_viz.c`. The headless `hazard_pathfinding.c` logs per-run metrics to `metrics.csv` for offline benchmarking.*
