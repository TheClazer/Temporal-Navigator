#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define ROWS 50
#define COLS 50
#define INF DBL_MAX
#define SAFETY_BUFFER 3.0

const int SCREEN_WIDTH = 1460;   // wide enough for a roomy full-height analysis rail
const int SCREEN_HEIGHT = 980;   // L-shaped layout: grid top-left, analysis strip below it
const int CELL_SIZE = 15;        // UNCHANGED: grid box stays x[50..800], y[50..800]

// ---------------------------------------------------------------------------
//  DAA Dashboard palette (dark analysis-tool theme)
// ---------------------------------------------------------------------------
// Surfaces
#define COL_BG            (Color){ 18,  20,  28, 255 }
#define COL_BG_GRID       (Color){ 24,  27,  38, 255 }
#define COL_PANEL         (Color){ 30,  34,  48, 255 }
#define COL_PANEL_HEAD    (Color){ 40,  46,  64, 255 }
#define COL_PANEL_LINE    (Color){ 58,  66,  90, 255 }
#define COL_BAR           (Color){ 22,  25,  35, 255 }
#define COL_GRIDLINE      (Color){ 40,  46,  64, 255 }
#define COL_WALL          (Color){ 96, 104, 130, 255 }
// Text
#define COL_TXT           (Color){ 232, 236, 245, 255 }
#define COL_TXT_DIM       (Color){ 182, 190, 208, 255 }
#define COL_TXT_FAINT     (Color){ 138, 148, 170, 255 }
// Algorithm accents (Dijkstra vs A* must read as distinct)
#define COL_DIJKSTRA      (Color){ 56,  150, 255, 255 }   // azure  -> Dijkstra
#define COL_ASTAR         (Color){ 168, 110, 255, 255 }   // violet -> A*
#define COL_DIJKSTRA_SOFT (Color){ 56,  150, 255,  70 }   // Dijkstra-only explored tint
#define COL_ASTAR_SOFT    (Color){ 168, 110, 255, 110 }   // A* explored / overlap tint
// Semantic
#define COL_HAZARD        (Color){ 240,  72,  66, 255 }
#define COL_HAZARD_SOFT   (Color){ 240,  72,  66,  45 }
#define COL_PATH          (Color){  46, 230, 160, 255 }
#define COL_FRONTIER      (Color){ 250, 204,  60, 255 }
#define COL_SUCCESS       (Color){  46, 230, 160, 255 }
#define COL_DANGER        (Color){ 240,  72,  66, 255 }
#define COL_WARN          (Color){ 250, 170,  60, 255 }
#define COL_START         (Color){  58, 170, 255, 255 }
#define COL_END           (Color){  46, 230, 160, 255 }

// ---------------------------------------------------------------------------
// 1. Structures and Globals
// ---------------------------------------------------------------------------

typedef struct {
    int base_cost;
    double hazard_arrival_time;
    int is_obstacle;
    int is_path;
    int x;
    int y;
} Cell;

Cell grid[ROWS][COLS];

typedef struct {
    int x;
    int y;
} Point;

typedef struct PathNode {
    int x;
    int y;
    struct PathNode *next;
} PathNode;

// Directions: 4-Way (Cardinal)
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

typedef struct {
    Point data[ROWS * COLS];
    int front;
    int rear;
} Queue;

// Global Visualization State
typedef enum { STATE_SETUP, STATE_WATERFLOW, STATE_SIMULATION, STATE_GAMEOVER } AppState;
AppState currentState = STATE_SETUP;

// Input Modes
typedef enum { INPUT_WALL, INPUT_HAZARD, INPUT_START, INPUT_END, INPUT_ERASER } InputMode;
InputMode currentInputMode = INPUT_WALL;

bool waterflowVisited[ROWS][COLS];
Queue waterflowQueue;
bool waterflowComplete = false;
bool mapIsSolvable = false;

// Global for Dijkstra Viz (Frontier)
bool dijkstraVisited[ROWS][COLS];

// Simulation Globals
Point startVal = {10, 5};
Point endVal = {10, 25};
Point hazards[100]; // Max 100 hazards for manual placement
int hazardCount = 0;
double current_time = 0.0;
PathNode* currentPlan = NULL;
Point agentPos;
int tick = 0;
int frameCounter = 0;
int replans = 0;
char eventMessage[256] = "SIMULATION START";
double currentRisk = 0.0;

// ---------------------------------------------------------------------------
// 1b. Algorithm instrumentation + A* / comparison state (DAA additions)
// ---------------------------------------------------------------------------

typedef struct {
    long nodesExpanded;   // # real nodes popped & processed (extractMin of a non-stale node)
    long edgesRelaxed;    // # successful relaxations (dist improved + decreaseKey)
    long heapPushes;      // # decreaseKey calls that INSERTED a new node (pos was -1)
    int  peakFrontier;    // max minHeap->size observed (space proxy)
    double pathCost;      // g-cost of the goal == dist[end] == steps (unit cost)
    int  pathLength;      // # cells in returned path
    bool success;         // did we reach the goal?
} AlgoStats;

// Active planner toggle (key 'A'): false = Dijkstra (default, uninformed), true = A* (informed)
bool useAstarPlanner = false;

// Live stats for whichever planner produced the current plan (drawn every frame)
AlgoStats liveStats = {0};

// Comparison-mode state (key 'C')
bool comparisonActive = false;
AlgoStats cmpDijkstra = {0};
AlgoStats cmpAstar    = {0};
bool dijkstraExplored[ROWS][COLS] = {0};   // explored set from the comparison Dijkstra run
bool astarExplored[ROWS][COLS]    = {0};   // explored set from the comparison A* run
bool cmpOptimalMatch = false;              // cmpDijkstra.pathCost == cmpAstar.pathCost

// UI font: a loaded anti-aliased TTF (Segoe UI / Consolas), with the raylib
// bitmap font as a last-resort fallback. All on-screen text renders through this.
Font g_font;

// ---------------------------------------------------------------------------
// 2. Queue & List Utils
// ---------------------------------------------------------------------------

void initQueue(Queue *q) {
    q->front = 0;
    q->rear = 0;
}

bool isQueueEmpty(Queue *q) {
    return q->front == q->rear;
}

void enqueue(Queue *q, Point p) {
    q->data[q->rear++] = p;
}

Point dequeue(Queue *q) {
    return q->data[q->front++];
}

PathNode* createPathNode(int x, int y) {
    PathNode* newNode = (PathNode*)malloc(sizeof(PathNode));
    newNode->x = x;
    newNode->y = y;
    newNode->next = NULL;
    return newNode;
}

void pushFront(PathNode** head, int x, int y) {
    PathNode* newNode = createPathNode(x, y);
    newNode->next = *head;
    *head = newNode;
}

void freePath(PathNode* head) {
    PathNode* tmp;
    while (head != NULL) {
        tmp = head;
        head = head->next;
        free(tmp);
    }
}

// ---------------------------------------------------------------------------
// 3. MinHeap
// ---------------------------------------------------------------------------

typedef struct {
    int x;
    int y;
    double dist;
} HeapNode;

typedef struct {
    HeapNode *data;
    int size;
    int capacity;
    int *pos;
} MinHeap;

MinHeap* createMinHeap(int capacity) {
    MinHeap *h = (MinHeap*)malloc(sizeof(MinHeap));
    h->data = (HeapNode*)malloc(capacity * sizeof(HeapNode));
    h->pos = (int*)malloc(ROWS * COLS * sizeof(int));
    h->size = 0;
    h->capacity = capacity;
    for (int i = 0; i < ROWS * COLS; i++) h->pos[i] = -1;
    return h;
}

void swapIndices(MinHeap *h, int i, int j) {
    HeapNode temp = h->data[i];
    h->data[i] = h->data[j];
    h->data[j] = temp;
    h->pos[h->data[i].x * COLS + h->data[i].y] = i;
    h->pos[h->data[j].x * COLS + h->data[j].y] = j;
}

void minHeapify(MinHeap *h, int idx) {
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;
    if (left < h->size && h->data[left].dist < h->data[smallest].dist) smallest = left;
    if (right < h->size && h->data[right].dist < h->data[smallest].dist) smallest = right;
    if (smallest != idx) {
        swapIndices(h, idx, smallest);
        minHeapify(h, smallest);
    }
}

bool isHeapEmpty(MinHeap *h) { return h->size == 0; }

HeapNode extractMin(MinHeap *h) {
    if (isHeapEmpty(h)) { HeapNode empty = {-1, -1, INF}; return empty; }
    HeapNode root = h->data[0];
    HeapNode lastNode = h->data[h->size - 1];
    h->data[0] = lastNode;
    h->pos[root.x * COLS + root.y] = -1;
    h->pos[lastNode.x * COLS + lastNode.y] = 0;
    h->size--;
    minHeapify(h, 0);
    return root;
}

void decreaseKey(MinHeap *h, int x, int y, double newDist) {
    int i = h->pos[x * COLS + y];
    if (i == -1) {
        i = h->size;
        h->size++;
        h->data[i].x = x;
        h->data[i].y = y;
        h->data[i].dist = newDist;
        h->pos[x * COLS + y] = i;
    }
    h->data[i].dist = newDist;
    while (i && h->data[(i - 1) / 2].dist > h->data[i].dist) {
        swapIndices(h, i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}

// ---------------------------------------------------------------------------
// 4. Algorithms
// ---------------------------------------------------------------------------

// Manhattan distance heuristic. Admissible AND consistent on a 4-connected,
// unit-cost grid: every cardinal step changes |dx|+|dy| by exactly 1 and costs
// exactly 1, so h never overestimates and satisfies h(u) <= w(u,v) + h(v).
static inline double heuristic(int x, int y, Point end) {
    int ddx = x - end.x; if (ddx < 0) ddx = -ddx;
    int ddy = y - end.y; if (ddy < 0) ddy = -ddy;
    return (double)(ddx + ddy);
}

void runHazardBFS(Point hazards[], int hazardCount) {
    Queue q;
    initQueue(&q);

    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            grid[i][j].hazard_arrival_time = INF;
            grid[i][j].x = i;
            grid[i][j].y = j;
            if (!grid[i][j].is_obstacle) grid[i][j].base_cost = 1;
        }
    }

    for (int k = 0; k < hazardCount; k++) {
        grid[hazards[k].x][hazards[k].y].hazard_arrival_time = 0;
        enqueue(&q, hazards[k]);
    }

    while (!isQueueEmpty(&q)) {
        Point curr = dequeue(&q);
        double current_time = grid[curr.x][curr.y].hazard_arrival_time;

        // 4-Way Propagation
        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];

            if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS && !grid[nx][ny].is_obstacle) {
                if (grid[nx][ny].hazard_arrival_time == INF) {
                    grid[nx][ny].hazard_arrival_time = current_time + 10.0;
                    Point next = {nx, ny};
                    enqueue(&q, next);
                }
            }
        }
    }
}

// Risk Calculation: Sum of how much we are "eating into" the safety buffer
double CalculateRisk(PathNode* path, double start_time) {
    double totalRisk = 0.0;
    int steps = 0;
    while(path) {
        double arrival = start_time + steps;
        double hazard = grid[path->x][path->y].hazard_arrival_time;

        if (hazard != INF) {
             double slack = hazard - arrival;
             if (slack < SAFETY_BUFFER) {
                 // If slack is negative (we are ON FIRE), risk is huge per tick
                 // SAFETY_BUFFER - slack. ex: SAFETY=3. Slack=-2. Risk+=5.
                 totalRisk += (SAFETY_BUFFER - slack);
             }
        }
        path = path->next;
        steps++;
    }
    return totalRisk;
}

// Unified Dijkstra / A* solver.
//   use_heuristic == false -> plain Dijkstra (byte-identical to the original)
//   use_heuristic == true  -> A* with Manhattan heuristic (heap keyed on f = g + h)
// dist[][] always holds the TRUE g-cost. The heap key may be f; never read g from
// the popped node. Pass stats==NULL to skip instrumentation, exploredOut==NULL to
// skip explored-set capture (the live run uses dijkstraVisited; comparison passes arrays).
PathNode* solve_unified(Point start, Point end, double start_time_offset, double safety_buffer,
                        bool ignore_safety_buffer, bool use_heuristic,
                        AlgoStats *stats, bool exploredOut[ROWS][COLS]) {
    // Reset live frontier viz only on the primary (safe-buffer) pass, exactly as before.
    if (!ignore_safety_buffer) {
        for(int i=0; i<ROWS; i++) for(int j=0; j<COLS; j++) dijkstraVisited[i][j] = false;
    }

    if (stats) {
        stats->nodesExpanded = 0;
        stats->edgesRelaxed  = 0;
        stats->heapPushes    = 0;
        stats->peakFrontier  = 0;
        stats->pathCost      = 0.0;
        stats->pathLength    = 0;
        stats->success       = false;
    }
    if (exploredOut) {
        for(int i=0; i<ROWS; i++) for(int j=0; j<COLS; j++) exploredOut[i][j] = false;
    }

    MinHeap *minHeap = createMinHeap(ROWS * COLS);
    double dist[ROWS][COLS];
    Point parent[ROWS][COLS];

    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            dist[i][j] = INF;
            parent[i][j] = (Point){-1, -1};
            grid[i][j].is_path = 0;
        }
    }

    dist[start.x][start.y] = 0.0;
    // Seed: g=0, so the key is h(start) for A*, 0 for Dijkstra.
    {
        double startKey = use_heuristic ? heuristic(start.x, start.y, end) : 0.0;
        decreaseKey(minHeap, start.x, start.y, startKey);   // INSERT
        if (stats) {
            stats->heapPushes++;
            if (minHeap->size > stats->peakFrontier) stats->peakFrontier = minHeap->size;
        }
    }

    PathNode* pathHead = NULL;

    while (!isHeapEmpty(minHeap)) {
        HeapNode u = extractMin(minHeap);
        int ux = u.x;
        int uy = u.y;

        // CORRECTNESS: read g from dist[][], NOT from the popped node.
        // For A*, u.dist is f = g + h. Only dist[][] holds the true g.
        double g_u = dist[ux][uy];

        // Stale-pop guard (defensive; does not count as an expansion).
        if (g_u == INF) {
            if (stats && minHeap->size > stats->peakFrontier) stats->peakFrontier = minHeap->size;
            continue;
        }

        dijkstraVisited[ux][uy] = true;                 // live frontier viz
        if (exploredOut) exploredOut[ux][uy] = true;    // comparison explored set
        if (stats) stats->nodesExpanded++;

        if (ux == end.x && uy == end.y) {
            // Reconstruct path
            int len = 0;
            Point curr = end;
            while (curr.x != -1) {
                pushFront(&pathHead, curr.x, curr.y);
                grid[curr.x][curr.y].is_path = 1;
                len++;
                curr = parent[curr.x][curr.y];
            }
            if (stats) {
                stats->success    = true;
                stats->pathCost   = dist[end.x][end.y];
                stats->pathLength = len;
            }
            free(minHeap->data);
            free(minHeap->pos);
            free(minHeap);
            return pathHead;
        }

        // 4-Way Neighbor Check
        for (int i = 0; i < 4; i++) {
            int vx = ux + dx[i];
            int vy = uy + dy[i];

            if (vx >= 0 && vx < ROWS && vy >= 0 && vy < COLS && !grid[vx][vy].is_obstacle) {
                double weight = (double)grid[vx][vy].base_cost;

                // Fallback Logic: penalty for hazards. Arrival uses g_u (NOT u.dist).
                if (ignore_safety_buffer) {
                     if (start_time_offset + g_u + weight >= grid[vx][vy].hazard_arrival_time) {
                         weight += 1000.0;
                     }
                }

                double new_dist = g_u + weight;          // new g for v

                bool canRelax = false;
                if (ignore_safety_buffer) {
                    canRelax = (new_dist < dist[vx][vy]);
                } else {
                    // Strict safety check: arrival (g) + buffer must beat hazard time. Uses g, not f.
                    if ((start_time_offset + new_dist + safety_buffer) < grid[vx][vy].hazard_arrival_time) {
                        canRelax = (new_dist < dist[vx][vy]);
                    }
                }

                if (canRelax) {
                    bool isNewInsert = (minHeap->pos[vx * COLS + vy] == -1);
                    dist[vx][vy] = new_dist;             // store pure g
                    parent[vx][vy] = (Point){ux, uy};

                    // Heap key: f for A*, g for Dijkstra. h added ONLY here.
                    double key = use_heuristic ? (new_dist + heuristic(vx, vy, end)) : new_dist;
                    decreaseKey(minHeap, vx, vy, key);

                    if (stats) {
                        stats->edgesRelaxed++;
                        if (isNewInsert) stats->heapPushes++;
                        if (minHeap->size > stats->peakFrontier) stats->peakFrontier = minHeap->size;
                    }
                }
            }
        }
    }

    free(minHeap->data);
    free(minHeap->pos);
    free(minHeap);
    return NULL;
}

// Back-compat wrapper: original 5-arg signature, plain Dijkstra, no stats/explored.
PathNode* solve_dijkstra(Point start, Point end, double start_time_offset,
                         double safety_buffer, bool ignore_safety_buffer) {
    return solve_unified(start, end, start_time_offset, safety_buffer,
                         ignore_safety_buffer, /*use_heuristic=*/false,
                         /*stats=*/NULL, /*exploredOut=*/NULL);
}

// ---------------------------------------------------------------------------
// 4b. Comparison mode: run BOTH planners on the frozen state, prove same cost
// ---------------------------------------------------------------------------

// solve_unified mutates grid[].is_path and dijkstraVisited as a side effect.
// Snapshot/restore them so a comparison run never disturbs the live plan/frontier.
static int  saved_is_path[ROWS][COLS];
static bool saved_visited[ROWS][COLS];

static void SaveSolverSideState(void) {
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++) {
            saved_is_path[i][j] = grid[i][j].is_path;
            saved_visited[i][j] = dijkstraVisited[i][j];
        }
}
static void RestoreSolverSideState(void) {
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++) {
            grid[i][j].is_path    = saved_is_path[i][j];
            dijkstraVisited[i][j] = saved_visited[i][j];
        }
}

// Run Dijkstra and A* from the live agent position on the CURRENT hazard state.
// Fills cmpDijkstra/cmpAstar + dijkstraExplored/astarExplored, leaves the live
// plan, frontier and path untouched, and verifies both return the same optimal cost.
void RunComparison(void) {
    Point cmpStart = agentPos;

    SaveSolverSideState();

    // Mirror the live planner's two-stage logic: prefer a strict-safe path; if none
    // exists, fall back to the risky (penalty) path for BOTH planners, so the
    // side-by-side reflects the same plan the agent is actually executing.
    PathNode* pD = solve_unified(cmpStart, endVal, current_time, SAFETY_BUFFER,
                                 /*ignore_safety_buffer=*/false, /*use_heuristic=*/false,
                                 &cmpDijkstra, dijkstraExplored);
    if (pD) freePath(pD);

    bool useFallback = !cmpDijkstra.success;
    if (useFallback) {
        PathNode* pDf = solve_unified(cmpStart, endVal, current_time, SAFETY_BUFFER,
                                      /*ignore_safety_buffer=*/true, /*use_heuristic=*/false,
                                      &cmpDijkstra, dijkstraExplored);
        if (pDf) freePath(pDf);
    }

    PathNode* pA = solve_unified(cmpStart, endVal, current_time, SAFETY_BUFFER,
                                 /*ignore_safety_buffer=*/useFallback, /*use_heuristic=*/true,
                                 &cmpAstar, astarExplored);
    if (pA) freePath(pA);

    RestoreSolverSideState();

    // Optimality assertion: if both succeed they MUST agree on cost (exact int steps).
    cmpOptimalMatch = (cmpDijkstra.success && cmpAstar.success &&
                       cmpDijkstra.pathCost == cmpAstar.pathCost);

    comparisonActive = true;
}

// ---------------------------------------------------------------------------
// 5. Map setup / reset
// ---------------------------------------------------------------------------

// Helper to restore default map state
void SetupDefaultMap() {
    // 1. Clear Grid
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            grid[i][j].base_cost = 1;
            grid[i][j].is_obstacle = 0;
            grid[i][j].hazard_arrival_time = INF;
            grid[i][j].is_path = 0;
        }
    }

    // 2. Default Wall Scenario: Wall at x=15, Gap at y=10
    for(int i=5; i<45; i++) {
        if(i != 10) grid[i][15].is_obstacle = 1;
    }

    // 3. Default Hazards
    hazardCount = 0;
    hazards[hazardCount++] = (Point){45, 45};
}

void SoftResetSimulation() {
    currentState = STATE_SETUP;
    current_time = 0.0;
    tick = 0;
    replans = 0;
    currentRisk = 0.0;
    frameCounter = 0;
    agentPos = startVal;
    sprintf(eventMessage, "SIMULATION RESET");

    // Clear dynamic hazard times but keep obstacles/hazards definitions
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            grid[i][j].hazard_arrival_time = INF;
            grid[i][j].is_path = 0;
             if (!grid[i][j].is_obstacle) grid[i][j].base_cost = 1;
        }
    }

    if (currentPlan) {
        freePath(currentPlan);
        currentPlan = NULL;
    }

    mapIsSolvable = false;
    waterflowComplete = false;
    comparisonActive = false;   // don't leave a stale comparison overlay after reset
    liveStats = (AlgoStats){0};
}

void ResetSimulation() {
    SetupDefaultMap(); // HARD RESET
    SoftResetSimulation();
}

void InitWaterflow() {
    for(int i=0; i<ROWS; i++) for(int j=0; j<COLS; j++) waterflowVisited[i][j] = false;
    initQueue(&waterflowQueue);
    enqueue(&waterflowQueue, startVal);
    waterflowVisited[startVal.x][startVal.y] = true;
    waterflowComplete = false;
    mapIsSolvable = false;
}

// ---------------------------------------------------------------------------
// 6. Input
// ---------------------------------------------------------------------------

void HandleInput() {
    // Mode Switching
    if (IsKeyPressed(KEY_ONE)) currentInputMode = INPUT_WALL;
    if (IsKeyPressed(KEY_TWO)) currentInputMode = INPUT_HAZARD;
    if (IsKeyPressed(KEY_THREE)) currentInputMode = INPUT_START;
    if (IsKeyPressed(KEY_FOUR)) currentInputMode = INPUT_END;
    if (IsKeyPressed(KEY_FIVE)) currentInputMode = INPUT_ERASER;

    // Simulation Control
    if (IsKeyPressed(KEY_R)) ResetSimulation(); // Hard Reset
    if (IsKeyPressed(KEY_BACKSPACE)) SoftResetSimulation(); // Soft Reset (Keep Map)

    // DAA: active planner toggle (Dijkstra <-> A*).
    if (IsKeyPressed(KEY_A)) {
        useAstarPlanner = !useAstarPlanner;
        // In a running sim, re-plan immediately from the current cell so the live
        // instrumentation reflects the newly selected planner right away.
        if (currentState == STATE_SIMULATION) {
            if (currentPlan) { freePath(currentPlan); currentPlan = NULL; }
            currentPlan = solve_unified(agentPos, endVal, current_time, SAFETY_BUFFER,
                                        false, useAstarPlanner, &liveStats, NULL);
            if (!currentPlan)
                currentPlan = solve_unified(agentPos, endVal, current_time, SAFETY_BUFFER,
                                            true, useAstarPlanner, &liveStats, NULL);
            currentRisk = currentPlan ? CalculateRisk(currentPlan, current_time) : INF;
        }
    }

    // DAA: comparison run (both planners, side-by-side overlay). Toggle on/off.
    if (IsKeyPressed(KEY_C)) {
        if (currentState == STATE_SIMULATION || currentState == STATE_GAMEOVER) {
            if (!comparisonActive) RunComparison();   // sets comparisonActive = true
            else comparisonActive = false;
        }
    }

    // State Transitions
    if (currentState == STATE_SETUP) {
        if (IsKeyPressed(KEY_ENTER)) {
             // 1. Hazard Pre-calc using current hazards list
             runHazardBFS(hazards, hazardCount);
             // 2. Start Waterflow
             currentState = STATE_WATERFLOW;
             InitWaterflow();
        }

        // Mouse Editing
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 mousePos = GetMousePosition();
            int c = (mousePos.x - 50) / CELL_SIZE;
            int r = (mousePos.y - 50) / CELL_SIZE;

            if (r >= 0 && r < ROWS && c >= 0 && c < COLS) {
                if (currentInputMode == INPUT_WALL) {
                    grid[r][c].is_obstacle = 1;
                    grid[r][c].hazard_arrival_time = INF; // Reset hazard if overwriting
                }
                else if (currentInputMode == INPUT_HAZARD) {
                    // Avoid duplicates
                    bool exists = false;
                    for(int k=0; k<hazardCount; k++) {
                        if (hazards[k].x == r && hazards[k].y == c) exists = true;
                    }
                    if (!exists && hazardCount < 100) {
                        hazards[hazardCount++] = (Point){r, c};
                    }
                }
                else if (currentInputMode == INPUT_START) {
                    startVal.x = r; startVal.y = c;
                    agentPos = startVal;
                }
                else if (currentInputMode == INPUT_END) {
                    endVal.x = r; endVal.y = c;
                }
                else if (currentInputMode == INPUT_ERASER) {
                     grid[r][c].is_obstacle = 0;
                     // Also remove hazard if present
                     int idx = -1;
                     for(int k=0; k<hazardCount; k++) if(hazards[k].x == r && hazards[k].y == c) idx = k;
                     if (idx != -1) {
                         hazards[idx] = hazards[hazardCount-1];
                         hazardCount--;
                     }
                }
            }
        }
        // Right Click to Clear Wall/Hazard
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 mousePos = GetMousePosition();
            int c = (mousePos.x - 50) / CELL_SIZE;
            int r = (mousePos.y - 50) / CELL_SIZE;

             if (r >= 0 && r < ROWS && c >= 0 && c < COLS) {
                 grid[r][c].is_obstacle = 0;

                 // Remove from hazards list if present
                 int idx = -1;
                 for(int k=0; k<hazardCount; k++) if(hazards[k].x == r && hazards[k].y == c) idx = k;
                 if (idx != -1) {
                     hazards[idx] = hazards[hazardCount-1]; // Swap with last
                     hazardCount--;
                 }
             }
        }
    }
    else if (currentState == STATE_WATERFLOW) {
         if (waterflowComplete && (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_ENTER))) {
             currentState = STATE_SIMULATION;
         }
    }
}

// ---------------------------------------------------------------------------
// 7. UI primitives (DAA dashboard chrome)
// ---------------------------------------------------------------------------

// Text helpers: render everything through the loaded TTF (g_font) so the
// dashboard is crisp and readable. Same call shape as DrawText/MeasureText.
static inline float g_spacing(int size) { return (float)size / 16.0f; }
static void DT(const char* t, int x, int y, int size, Color c) {
    DrawTextEx(g_font, t, (Vector2){ (float)x, (float)y }, (float)size, g_spacing(size), c);
}
static int MT(const char* t, int size) {
    return (int)MeasureTextEx(g_font, t, (float)size, g_spacing(size)).x;
}

// A titled card with rounded body + accent title bar. Returns the Y of the first content line.
static int DrawPanel(int x, int y, int w, int h, const char* title, Color accent) {
    Rectangle body = { (float)x, (float)y, (float)w, (float)h };
    DrawRectangleRounded(body, 0.05f, 6, COL_PANEL);
    DrawRectangleRoundedLines(body, 0.05f, 6, 1.0f, COL_PANEL_LINE); // raylib 5.0: 5-arg
    if (title != NULL) {
        const int TITLE_H = 30;
        DrawRectangle(x + 2, y + 2, w - 4, TITLE_H, COL_PANEL_HEAD);
        DrawRectangle(x + 2, y + 2, 4, TITLE_H, accent);                 // accent tab
        DrawRectangle(x + 2, y + TITLE_H + 1, w - 4, 2, accent);         // accent underline
        DT(title, x + 16, y + 8, 17, COL_TXT);
        return y + TITLE_H + 14;
    }
    return y + 14;
}

// Left label (dim) + right-aligned value (bright).
static void DrawStatRow(int x, int y, int rowW, const char* label, const char* value, Color valColor) {
    DT(label, x, y, 17, COL_TXT_DIM);
    int vw = MT(value, 17);
    DT(value, x + rowW - vw, y, 17, valColor);
}

// A small pill/badge. Returns its width so badges can be chained.
static int DrawBadge(int x, int y, const char* text, Color fill, Color txt) {
    int tw = MT(text, 16);
    int padX = 12, w = tw + padX * 2, h = 28;
    Rectangle r = { (float)x, (float)y, (float)w, (float)h };
    DrawRectangleRounded(r, 0.5f, 6, fill);
    DT(text, x + padX, y + 6, 16, txt);
    return w;
}

// Legend swatch + label.
static void DrawLegendItem(int x, int y, Color swatch, const char* label) {
    DrawRectangle(x, y + 1, 15, 15, swatch);
    DrawRectangleLines(x, y + 1, 15, 15, COL_PANEL_LINE);
    DT(label, x + 23, y, 15, COL_TXT_DIM);
}

static int CountWaterVisited(void) {
    int n = 0;
    for (int i = 0; i < ROWS; i++) for (int j = 0; j < COLS; j++) if (waterflowVisited[i][j]) n++;
    return n;
}

// ---------------------------------------------------------------------------
// 8. Top banner
// ---------------------------------------------------------------------------

static void DrawBanner(void) {
    DrawRectangle(0, 0, SCREEN_WIDTH, 50, COL_BG_GRID);
    DrawRectangle(0, 49, SCREEN_WIDTH, 1, COL_PANEL_LINE);

    DT("TEMPORAL NAVIGATOR", 50, 5, 26, COL_TXT);
    DT("Design & Analysis of Algorithms  -  Pathfinding under Dynamic Hazards",
             52, 33, 13, COL_TXT_DIM);

    // Active-algorithm badge (right-aligned)
    const char* algoName = useAstarPlanner ? "A*  (INFORMED)" : "DIJKSTRA  (UNINFORMED)";
    Color accent = useAstarPlanner ? COL_ASTAR : COL_DIJKSTRA;
    int tw = MT(algoName, 15);
    int badgeW = tw + 42;
    int bx = SCREEN_WIDTH - badgeW - 50;
    int by = 10;
    Rectangle br = { (float)bx, (float)by, (float)badgeW, 28 };
    DrawRectangleRounded(br, 0.5f, 6, accent);
    DrawCircle(bx + 18, by + 14, 5, COL_BG);
    DT(algoName, bx + 30, by + 6, 15, COL_BG);
    const char* lbl = "ACTIVE PLANNER";
    DT(lbl, bx - MT(lbl, 12) - 14, by + 8, 12, COL_TXT_FAINT);
}

// ---------------------------------------------------------------------------
// 9. Grid renderers (recolored for the dark theme)
// ---------------------------------------------------------------------------

void DrawSetupGrid(void) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            int posX = j * CELL_SIZE + 50;
            int posY = i * CELL_SIZE + 50;
            if (grid[i][j].is_obstacle) DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_WALL);
            else DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, COL_GRIDLINE);

            for (int k = 0; k < hazardCount; k++)
                if (hazards[k].x == i && hazards[k].y == j)
                    DrawCircle(posX + CELL_SIZE/2, posY + CELL_SIZE/2, CELL_SIZE/3, COL_HAZARD);
        }
    }
    DrawRectangle(startVal.y * CELL_SIZE + 50, startVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, COL_START);
    DrawRectangle(endVal.y   * CELL_SIZE + 50, endVal.x   * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, COL_END);
}

void DrawWaterflowRender() {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            int posX = j * CELL_SIZE + 50;
            int posY = i * CELL_SIZE + 50;

            if (grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_WALL);
            } else if (waterflowVisited[i][j]) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, Fade(COL_DIJKSTRA, 0.20f));
                DrawCircle(posX + CELL_SIZE/2, posY + CELL_SIZE/2, 2, COL_DIJKSTRA);
            } else {
                DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, COL_GRIDLINE);
            }
        }
    }
    DrawRectangle(startVal.y * CELL_SIZE + 50, startVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(COL_START, 0.7f));
    DrawRectangle(endVal.y   * CELL_SIZE + 50, endVal.x   * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(COL_END, 0.7f));
}

void DrawMap(double current_time) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            int posX = j * CELL_SIZE + 50;
            int posY = i * CELL_SIZE + 50;

            if (grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_WALL);
            } else {
                 DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, COL_GRIDLINE);
            }

            // Search frontier (explored / closed set)
            if (dijkstraVisited[i][j] && !grid[i][j].is_path && !grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, Fade(COL_FRONTIER, 0.28f));
            }

            // Hazards
            if (!grid[i][j].is_obstacle) {
                 if (grid[i][j].hazard_arrival_time <= current_time) {
                    DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_HAZARD);
                 } else if (grid[i][j].hazard_arrival_time < INF && grid[i][j].hazard_arrival_time > current_time) {
                     DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_HAZARD_SOFT);
                 }
            }

            // Committed path
            if (grid[i][j].is_path && grid[i][j].hazard_arrival_time > current_time) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_PATH);
            }
        }
    }
}

// Comparison overlay: tint Dijkstra-only explored cells vs A* explored cells so the
// viewer SEES A*'s tighter cone inside Dijkstra's wider flood.
void DrawComparisonOverlay(void) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (grid[i][j].is_obstacle) continue;
            int posX = j * CELL_SIZE + 50;
            int posY = i * CELL_SIZE + 50;
            bool d = dijkstraExplored[i][j];
            bool a = astarExplored[i][j];
            if (a)              DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_ASTAR_SOFT);
            else if (d && !a)   DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_DIJKSTRA_SOFT);
        }
    }
    // Keep the committed path readable on top of the tints.
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++)
            if (grid[i][j].is_path) {
                int posX = j*CELL_SIZE+50, posY = i*CELL_SIZE+50;
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, COL_PATH);
            }
}

// ---------------------------------------------------------------------------
// 10. Right rail (card stack) + bottom analysis bar
// ---------------------------------------------------------------------------

void DrawRightRail(void) {
    int cardX = 832, cardW = 600, lx = cardX + 16, rowW = cardW - 32;
    Color accent = useAstarPlanner ? COL_ASTAR : COL_DIJKSTRA;

    // ---- CARD 1: MISSION STATE -------------------------------------------------
    int c1 = DrawPanel(cardX, 60, cardW, 86, "MISSION STATE", COL_FRONTIER);
    if (currentState == STATE_SETUP) {
        DT("SETUP - Map Editor", lx, c1, 20, COL_TXT);
        DT("Draw the world, then commit with [ENTER].", lx, c1 + 26, 14, COL_TXT_FAINT);
    } else if (currentState == STATE_WATERFLOW) {
        DT("WATERFLOW - BFS Reachability", lx, c1, 18, COL_TXT);
        DT("Flood-fill checks the goal is reachable.", lx, c1 + 26, 14, COL_TXT_FAINT);
    } else {
        Color sc = COL_SUCCESS; const char* st = "NORMAL";
        if (tick > 5 && tick < 12 && replans > 0) { sc = COL_DANGER; st = "CRITICAL / REPLAN"; }
        if (currentState == STATE_GAMEOVER) { sc = COL_SUCCESS; st = "MISSION COMPLETE"; }
        DT(TextFormat("RUNNING   |   t = %.0f", current_time), lx, c1, 19, COL_TXT);
        DT(st, lx, c1 + 26, 15, sc);
    }

    // ---- CARD 2: ACTIVE ALGORITHM ---------------------------------------------
    int c2 = DrawPanel(cardX, 156, cardW, 100, "ACTIVE ALGORITHM", accent);
    DT("[A] switch planner    |    [C] compare both", lx, c2, 14, COL_TXT_DIM);
    int bw = DrawBadge(lx, c2 + 26, "DIJKSTRA",
                       useAstarPlanner ? COL_PANEL_HEAD : COL_DIJKSTRA,
                       useAstarPlanner ? COL_TXT_DIM    : COL_BG);
    DrawBadge(lx + bw + 12, c2 + 26, "A*",
              useAstarPlanner ? COL_ASTAR : COL_PANEL_HEAD,
              useAstarPlanner ? COL_BG    : COL_TXT_DIM);

    // ---- CARD 3: COMPLEXITY INSTRUMENTATION (live empirical counters) ----------
    int c3 = DrawPanel(cardX, 266, cardW, 244, "COMPLEXITY INSTRUMENTATION", COL_PATH);
    int sy = c3, step = 26;
    if (currentState == STATE_SIMULATION || currentState == STATE_GAMEOVER) {
        DrawStatRow(lx, sy,           rowW, "Nodes expanded", TextFormat("%ld", liveStats.nodesExpanded), accent);
        DrawStatRow(lx, sy + step,    rowW, "Edges relaxed",  TextFormat("%ld", liveStats.edgesRelaxed),  COL_TXT);
        DrawStatRow(lx, sy + step*2,  rowW, "Heap pushes",    TextFormat("%ld", liveStats.heapPushes),    COL_TXT);
        DrawStatRow(lx, sy + step*3,  rowW, "Peak frontier (space)", TextFormat("%d", liveStats.peakFrontier), COL_FRONTIER);
        DrawStatRow(lx, sy + step*4,  rowW, "Path cost (steps)", TextFormat("%.0f", liveStats.pathCost),  COL_PATH);
        DrawStatRow(lx, sy + step*5,  rowW, "Path length (cells)", TextFormat("%d", liveStats.pathLength), COL_PATH);
        Color rkc = COL_SUCCESS; char rks[32];
        if (currentRisk >= DBL_MAX || currentRisk > 100000) { snprintf(rks, sizeof rks, "EXTREME"); rkc = COL_DANGER; }
        else { snprintf(rks, sizeof rks, "%.1f", currentRisk); if (currentRisk > 5) rkc = COL_WARN; if (currentRisk > 20) rkc = COL_DANGER; }
        DrawStatRow(lx, sy + step*6,  rowW, TextFormat("Replans: %d   |   Buffer risk", replans), rks, rkc);
    } else if (currentState == STATE_WATERFLOW) {
        DrawStatRow(lx, sy,          rowW, "Algorithm", "BFS (uninformed)", COL_TXT);
        DrawStatRow(lx, sy + step,   rowW, "Queue size", TextFormat("%d", waterflowQueue.rear - waterflowQueue.front), COL_DIJKSTRA);
        DrawStatRow(lx, sy + step*2, rowW, "Cells visited", TextFormat("%d / %d", CountWaterVisited(), ROWS*COLS), COL_TXT);
        const char* rch = waterflowComplete ? (mapIsSolvable ? "CONFIRMED" : "IMPOSSIBLE") : "ANALYZING...";
        Color rcc = waterflowComplete ? (mapIsSolvable ? COL_SUCCESS : COL_DANGER) : COL_WARN;
        DrawStatRow(lx, sy + step*3, rowW, "Reachability", rch, rcc);
        if (waterflowComplete && mapIsSolvable)
            DT("[SPACE] start simulation", lx, sy + step*5, 17, COL_SUCCESS);
    } else {
        DrawStatRow(lx, sy,          rowW, "Nodes expanded", "-", COL_TXT_FAINT);
        DrawStatRow(lx, sy + step,   rowW, "Edges relaxed",  "-", COL_TXT_FAINT);
        DrawStatRow(lx, sy + step*2, rowW, "Heap pushes",    "-", COL_TXT_FAINT);
        DrawStatRow(lx, sy + step*3, rowW, "Peak frontier",  "-", COL_TXT_FAINT);
        DrawStatRow(lx, sy + step*4, rowW, "Path cost",      "-", COL_TXT_FAINT);
        DT("Commit a map and run to populate metrics.", lx, sy + step*5 + 6, 14, COL_TXT_FAINT);
    }

    // ---- CARD 4: COMPLEXITY REFERENCE (theory) ---------------------------------
    int c4 = DrawPanel(cardX, 520, cardW, 222, "COMPLEXITY REFERENCE  (theory)", COL_DIJKSTRA);
    int ry = c4, col2 = lx + 112;
    DT(TextFormat("V = R*C = %d cells      E = %d (~4V)", ROWS*COLS,
                        2*(ROWS*(COLS-1) + COLS*(ROWS-1))), lx, ry, 14, COL_TXT_DIM);
    DT("BFS", lx, ry+24, 16, COL_TXT);            DT("O(V+E)          space O(V)",  col2, ry+24, 16, COL_TXT_DIM);
    DT("Dijkstra", lx, ry+48, 16, COL_DIJKSTRA);  DT("O((V+E) log V)  space O(V)",  col2, ry+48, 16, COL_TXT_DIM);
    DT("A*", lx, ry+72, 16, COL_ASTAR);           DT("O((V+E) log V)* space O(V)",  col2, ry+72, 16, COL_TXT_DIM);
    DT("Heap op", lx, ry+96, 16, COL_TXT);        DT("push/extract/decrease O(logV)", col2, ry+96, 16, COL_TXT_DIM);
    DT("* admissible h = Manhattan  =>  A* optimal,", lx, ry+126, 14, COL_TXT_FAINT);
    DT("  expands fewer nodes than Dijkstra ([C]).", lx, ry+146, 14, COL_TXT_FAINT);

    // ---- CARD 5: LEGEND & CONTROLS --------------------------------------------
    int c5 = DrawPanel(cardX, 752, cardW, 208, "LEGEND & CONTROLS", COL_TXT_DIM);
    int ly = c5;
    if (currentState == STATE_SETUP) {
        const char* tools[5] = {"[1] Wall", "[2] Hazard", "[3] Start", "[4] End", "[5] Eraser"};
        for (int k = 0; k < 5; k++) {
            Color tc = (currentInputMode == (InputMode)k) ? COL_FRONTIER : COL_TXT_DIM;
            DT(tools[k], lx + (k%3)*190, ly + (k/3)*26, 16, tc);
        }
        DrawLegendItem(lx, ly + 60, COL_START, "Start");
        DrawLegendItem(lx + 150, ly + 60, COL_END, "End");
        DrawLegendItem(lx + 300, ly + 60, COL_HAZARD, "Hazard");
        DT("LMB place   |   RMB clear   |   [ENTER] commit map", lx, ly + 92, 14, COL_TXT_FAINT);
        DT("[R] hard reset    |    [BKSP] soft reset", lx, ly + 116, 14, COL_TXT_FAINT);
    } else {
        DrawLegendItem(lx, ly,      COL_FRONTIER, "Search frontier (explored)");
        DrawLegendItem(lx, ly + 24, COL_PATH,     "Agent path (safe)");
        DrawLegendItem(lx, ly + 48, COL_HAZARD,   "Active hazard");
        DrawLegendItem(lx + 290, ly + 48, COL_HAZARD_SOFT, "Future heat");
        DrawLegendItem(lx, ly + 72, COL_DIJKSTRA_SOFT, "Dijkstra-only explored ([C])");
        DrawLegendItem(lx, ly + 96, COL_ASTAR_SOFT, "A* explored / overlap ([C])");
        DT("[A] toggle algorithm   |   [C] compare    [R]/[BKSP] reset", lx, ly + 124, 13, COL_TXT_FAINT);
    }
}

// Bottom-left analysis strip (under the grid): empirical-vs-theory on top, then the
// live data-structure viz (left) and event log (right). Sits in x[0..808], y[808..bottom].
void DrawAnalysisBar(void) {
    int top = 808, barW = 808;
    DrawRectangle(0, top, barW, SCREEN_HEIGHT - top, COL_BAR);
    DrawRectangle(0, top, barW, 1, COL_PANEL_LINE);
    DrawRectangle(barW, top, 1, SCREEN_HEIGHT - top, COL_PANEL_LINE);

    Color accent = useAstarPlanner ? COL_ASTAR : COL_DIJKSTRA;

    // --- Row A: empirical vs theoretical (full width) -----------------------
    DT("ALGORITHM ANALYSIS  -  empirical vs theory", 24, top + 12, 15, COL_TXT_DIM);
    DT(useAstarPlanner ? "A*  -  O((V+E) log V) worst case; far fewer expansions in practice"
                       : "DIJKSTRA  -  O((V+E) log V)", 24, top + 36, 16, accent);
    float pct = (100.0f * (float)liveStats.nodesExpanded) / (float)(ROWS*COLS);
    DT(TextFormat("V=%d   E=%d   |   expanded=%ld   relaxed=%ld   |   frontier touched %.1f%%",
                  ROWS*COLS, 2*(ROWS*(COLS-1)+COLS*(ROWS-1)),
                  liveStats.nodesExpanded, liveStats.edgesRelaxed, pct), 24, top + 62, 14, COL_TXT);

    // --- Row B: data structure (left) + event log (right) -------------------
    int by = top + 92;
    DrawRectangle(24, by - 4, barW - 48, 1, COL_PANEL_LINE);
    if (currentState == STATE_WATERFLOW) {
        DT("DATA STRUCTURE - Queue (FIFO)", 24, by, 14, COL_TXT_DIM);
        int bs = 22, gap = 5, count = 0;
        for (int i = waterflowQueue.front; i < waterflowQueue.rear && count < 9; i++, count++) {
            int x = 24 + count * (bs + gap), yy = by + 24;
            DrawRectangle(x, yy, bs, bs, Fade(COL_DIJKSTRA, 0.30f));
            DrawRectangleLines(x, yy, bs, bs, COL_DIJKSTRA);
            if (i == waterflowQueue.front) DT("H", x + 6, yy + 4, 13, COL_TXT);
        }
        DT(TextFormat("size = %d   (head -> tail)", waterflowQueue.rear - waterflowQueue.front), 24, by + 54, 13, COL_TXT_FAINT);
    } else if (currentState == STATE_SIMULATION || currentState == STATE_GAMEOVER) {
        DT("DATA STRUCTURE - Linked List (plan)", 24, by, 14, COL_TXT_DIM);
        PathNode* t = currentPlan; int bs = 22, gap = 16, count = 0;
        while (t && count < 6) {
            int x = 24 + count * (bs + gap), yy = by + 24;
            DrawRectangle(x, yy, bs, bs, Fade(COL_PATH, 0.35f));
            DrawRectangleLines(x, yy, bs, bs, COL_PATH);
            if (t->next && count < 5) DrawLine(x + bs, yy + bs/2, x + bs + gap, yy + bs/2, COL_TXT_DIM);
            t = t->next; count++;
        }
        if (t) DT("...", 24 + count * (bs + gap), by + 28, 18, COL_TXT_DIM);
        DT("agent's planned path (next cells)", 24, by + 54, 13, COL_TXT_FAINT);
    } else {
        DT("DATA STRUCTURE - live", 24, by, 14, COL_TXT_DIM);
        DT("Queue (BFS) and Linked List (path) appear", 24, by + 26, 13, COL_TXT_FAINT);
        DT("once the simulation runs.", 24, by + 46, 13, COL_TXT_FAINT);
    }

    // Event log (right portion of row B)
    int rx = 430;
    DrawRectangle(rx - 16, by - 4, 1, SCREEN_HEIGHT - (by - 4), COL_PANEL_LINE);
    DT("EVENT LOG", rx, by, 14, COL_TXT_DIM);
    const char* msg = (currentState == STATE_SIMULATION || currentState == STATE_GAMEOVER)
                      ? eventMessage : "Awaiting simulation...";
    DT(msg, rx, by + 26, 16, COL_WARN);
    if (comparisonActive) DT("COMPARISON MODE ACTIVE - see overlay", rx, by + 52, 13, COL_FRONTIER);
}

// Floating head-to-head results card (shown over the grid during comparison).
void DrawComparisonCard(void) {
    int x = 90, y = 200, w = 680, h = 360;
    int cy = DrawPanel(x, y, w, h, "DIJKSTRA  vs  A*    -    HEAD TO HEAD", COL_FRONTIER);
    int lx = x + 20, colD = x + 380, colA = x + 540;

    DT("METRIC", lx, cy, 15, COL_TXT_DIM);
    DT("DIJKSTRA", colD, cy, 15, COL_DIJKSTRA);
    DT("A*", colA, cy, 15, COL_ASTAR);
    cy += 30;

    const char* labels[6] = {"Nodes expanded","Edges relaxed","Heap pushes","Peak frontier","Path cost","Path length"};
    long dv[6] = {cmpDijkstra.nodesExpanded, cmpDijkstra.edgesRelaxed, cmpDijkstra.heapPushes,
                  cmpDijkstra.peakFrontier, (long)cmpDijkstra.pathCost, cmpDijkstra.pathLength};
    long av[6] = {cmpAstar.nodesExpanded, cmpAstar.edgesRelaxed, cmpAstar.heapPushes,
                  cmpAstar.peakFrontier, (long)cmpAstar.pathCost, cmpAstar.pathLength};
    for (int i = 0; i < 6; i++) {
        DT(labels[i], lx, cy, 17, COL_TXT_DIM);
        DT(TextFormat("%ld", dv[i]), colD, cy, 17, COL_TXT);
        DT(TextFormat("%ld", av[i]), colA, cy, 17, COL_TXT);
        cy += 28;
    }
    cy += 8;
    if (cmpDijkstra.success && cmpAstar.success) {
        long saved = cmpDijkstra.nodesExpanded - cmpAstar.nodesExpanded;
        double pct = cmpDijkstra.nodesExpanded ? 100.0*(double)saved/(double)cmpDijkstra.nodesExpanded : 0.0;
        DT(cmpOptimalMatch ? "OPTIMALITY: VERIFIED - identical optimal cost (both correct)"
                                 : "OPTIMALITY: MISMATCH (check admissibility)",
                 lx, cy, 16, cmpOptimalMatch ? COL_SUCCESS : COL_DANGER);
        if (saved > 0)
            DT(TextFormat("A* expanded %ld fewer nodes  (%.1f%% reduction). Heuristic pays off.", saved, pct),
                     lx, cy + 26, 16, COL_FRONTIER);
        else if (saved < 0)
            DT(TextFormat("A* expanded %ld more nodes here (tie-break overhead) - cost still optimal.", -saved),
                     lx, cy + 26, 15, COL_WARN);
        else
            DT("Equal expansions: forced/funnel map - the heuristic cannot prune here.",
                     lx, cy + 26, 15, COL_WARN);
    } else {
        DT("NO SAFE PATH from current cell - comparison N/A.", lx, cy, 16, COL_DANGER);
    }
    DT("[C] dismiss", lx, y + h - 28, 14, COL_TXT_FAINT);
}

// ---------------------------------------------------------------------------
// 11. Waterflow update
// ---------------------------------------------------------------------------

void UpdateWaterflow(Point end) {
    if (waterflowComplete) {
        return;
    }

    // Process multiple nodes per frame for faster yet visible animation
    int expansions = 0;
    while (!isQueueEmpty(&waterflowQueue) && expansions < 30) {
        Point curr = dequeue(&waterflowQueue);

        if (curr.x == end.x && curr.y == end.y) {
            mapIsSolvable = true;
            waterflowComplete = true;
        }

        // 4-Way Flood Fill
        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];
            if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS &&
                !grid[nx][ny].is_obstacle && !waterflowVisited[nx][ny]) {
                waterflowVisited[nx][ny] = true;
                enqueue(&waterflowQueue, (Point){nx, ny});
            }
        }
        expansions++;
    }

    if (isQueueEmpty(&waterflowQueue)) waterflowComplete = true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Temporal Navigator - Design & Analysis of Algorithms");
    SetTargetFPS(60);

    // Load a crisp anti-aliased UI font (downscaled from a large base size for sharpness).
    // Prefer Segoe UI, then Consolas; fall back to raylib's built-in font if neither exists.
    g_font = LoadFontEx("C:\\Windows\\Fonts\\segoeui.ttf", 48, NULL, 0);
    if (g_font.texture.id == 0) g_font = LoadFontEx("C:\\Windows\\Fonts\\consola.ttf", 48, NULL, 0);
    if (g_font.texture.id == 0) g_font = GetFontDefault();
    SetTextureFilter(g_font.texture, TEXTURE_FILTER_BILINEAR);

    // Initial Setup
    agentPos = startVal;
    ResetSimulation(); // Calls SetupDefaultMap inside
    agentPos = startVal;
    ResetSimulation();

    while (!WindowShouldClose()) {

        HandleInput();

        // --- Logic ---
        if (currentState == STATE_WATERFLOW) {
            UpdateWaterflow(endVal);
        }
        else if (currentState == STATE_SIMULATION) {

            // On First Frame of Sim, Plan (with the active planner)
            if (currentPlan == NULL && tick == 0) {
                 // Try Safe Path
                 currentPlan = solve_unified(startVal, endVal, current_time, SAFETY_BUFFER, false, useAstarPlanner, &liveStats, NULL);
                 if (currentPlan) {
                     currentRisk = 0.0;
                 } else {
                     // Fallback: Risky Path
                     currentPlan = solve_unified(startVal, endVal, current_time, SAFETY_BUFFER, true, useAstarPlanner, &liveStats, NULL);
                     if (currentPlan) currentRisk = CalculateRisk(currentPlan, current_time);
                     else {
                         currentRisk = INF;
                     }
                 }
            }

            frameCounter++;
            // Simulation Speed: Update every 10 frames
            // Freeze the agent while a comparison overlay is being inspected, so the
            // frozen explored-sets / head-to-head numbers stay truthful to agentPos.
            if (frameCounter >= 10 && agentPos.x != -1 && currentState != STATE_GAMEOVER && !comparisonActive) {
                 frameCounter = 0;

                 // Dynamic Event at Tick 5
                 if (tick == 5) {
                    grid[10][15].hazard_arrival_time = 0;
                    grid[9][15].hazard_arrival_time = 0;
                    grid[11][15].hazard_arrival_time = 0;
                    sprintf(eventMessage, "WALL COLLAPSE! FIRE SPREADING!");
                 }

                 // Controller
                 bool pathValid = true;
                 PathNode* temp = currentPlan;
                 int steps = 0;
                 while(temp) {
                     // Check STRICT survival (no buffer) just to see if path is still physically valid
                     if (current_time + steps >= grid[temp->x][temp->y].hazard_arrival_time) {
                         pathValid = false; break;
                     }
                     temp = temp->next; steps++;
                 }

                 if (!pathValid) {
                     if(currentPlan) freePath(currentPlan);

                     // Try Safe
                     currentPlan = solve_unified(agentPos, endVal, current_time, SAFETY_BUFFER, false, useAstarPlanner, &liveStats, NULL);
                     if (currentPlan) {
                         currentRisk = 0.0;
                     } else {
                         // Fallback
                         currentPlan = solve_unified(agentPos, endVal, current_time, SAFETY_BUFFER, true, useAstarPlanner, &liveStats, NULL);
                         if(currentPlan) currentRisk = CalculateRisk(currentPlan, current_time);
                         else currentRisk = INF;
                     }
                     replans++;
                 }

                 // Recalculate Risk on every step (it changes as we get closer/farther or as events happen)
                 if(currentPlan) currentRisk = CalculateRisk(currentPlan, current_time);

                 // Move
                 if (currentPlan && currentPlan->x == agentPos.x && currentPlan->y == agentPos.y) {
                     PathNode* old = currentPlan;
                     currentPlan = currentPlan->next;
                     free(old);
                 }
                 if (currentPlan) {
                     agentPos.x = currentPlan->x;
                     agentPos.y = currentPlan->y;
                 }

                 current_time += 1.0;
                 tick++;

                 if(agentPos.x == endVal.x && agentPos.y == endVal.y) {
                     currentState = STATE_GAMEOVER;
                 }
            }
        }

        // --- Draw ---
        BeginDrawing();
            ClearBackground(COL_BG);

            // Grid backdrop + decorative frame (grid box itself is unchanged)
            DrawRectangle(50, 50, COLS*CELL_SIZE, ROWS*CELL_SIZE, COL_BG_GRID);
            DrawRectangleLinesEx((Rectangle){46, 46, COLS*CELL_SIZE + 8.0f, ROWS*CELL_SIZE + 8.0f}, 2, COL_PANEL_LINE);

            DrawBanner();

            if (currentState == STATE_SETUP) {
                DrawSetupGrid();
            }
            else if (currentState == STATE_WATERFLOW) {
                DrawWaterflowRender();
            }
            else {
                // Sim / Gameover
                DrawMap(current_time);

                if (comparisonActive) DrawComparisonOverlay();

                DrawRectangle(agentPos.y * CELL_SIZE + 50, agentPos.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, COL_START);
                DrawRectangle(endVal.y   * CELL_SIZE + 50, endVal.x   * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(COL_END, 0.5f));

                if (currentState == STATE_GAMEOVER) {
                    DrawBadge(290, 62, "MISSION SUCCESS", COL_SUCCESS, COL_BG);
                }
            }

            DrawRightRail();
            DrawAnalysisBar();

            if (comparisonActive) DrawComparisonCard();

        EndDrawing();
    }

    if (currentPlan) { freePath(currentPlan); currentPlan = NULL; }
    CloseWindow();
    return 0;
}
