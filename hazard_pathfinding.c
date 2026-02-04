#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#define ROWS 50
#define COLS 50
#define SAFETY_BUFFER 5.0
#define INF DBL_MAX

// ---------------------------------------------------------------------------
// 1. Structures and Globals
// ---------------------------------------------------------------------------

typedef struct {
    int base_cost;
    double hazard_arrival_time;
    int is_obstacle;
    int is_path;             // New field: 1 if part of final path, 0 otherwise
    int x;
    int y;
} Cell;

Cell grid[ROWS][COLS];

typedef struct {
    int x;
    int y;
} Point;

// Linked List Node for Path
typedef struct PathNode {
    int x;
    int y;
    struct PathNode *next;
} PathNode;

// Directions: Up, Down, Left, Right
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

typedef struct {
    Point data[ROWS * COLS];
    int front;
    int rear;
} Queue;

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

// Prepend for O(1) insertion during backtracking
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
// 3. Helper Utils (Sleep, ANSI, Logging)
// ---------------------------------------------------------------------------

void sleep_ms(int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

#define ANSI_COLOR_RED      "\x1b[31m"
#define ANSI_COLOR_GREEN    "\x1b[32m"
#define ANSI_COLOR_YELLOW   "\x1b[33m"
#define ANSI_COLOR_CYAN     "\x1b[36m"
#define ANSI_COLOR_WHITE    "\x1b[37m" // Explicit White
#define ANSI_COLOR_GRAY     "\x1b[90m"
#define ANSI_COLOR_RESET    "\x1b[0m"

// Global to store ghost path for persistence
Point ghostPath[ROWS * COLS];
int ghostPathLen = 0;
int ghostFramesLeft = 0;

void render_map(Point currentPath[], int pathLen, double current_time, int tick, int replans, char* mode, double safety_margin) {
    printf("\033[H"); // Reset cursor
    
    // --- DASHBOARD HEADER ---
    printf(ANSI_COLOR_WHITE "==================================================\n");
    printf(" TICK: %-4d | MODE: %-10s | MARGIN: %.1fs\n", tick, mode, safety_margin);
    printf(" REPLANS: %-2d | STATUS: %s\n", replans, (replans > 0 && tick == 5) ? ANSI_COLOR_RED "CRITICAL EVENT" ANSI_COLOR_RESET : ANSI_COLOR_GREEN "NOMINAL" ANSI_COLOR_RESET);
    printf("==================================================" ANSI_COLOR_RESET "\n");

    // Update ghost path logic (same as before)
    // ...
    
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            bool isCurrent = false;
            for(int k=0; k<pathLen; k++) {
                if (currentPath[k].x == i && currentPath[k].y == j) {
                    isCurrent = true;
                    break;
                }
            }
            
            bool isGhost = false;
            // Ghost logic same...
            if (!isCurrent && ghostFramesLeft > 0) {
                 for(int k=0; k<ghostPathLen; k++) { if(ghostPath[k].x == i && ghostPath[k].y == j) { isGhost=true; break; } }
            }

            if (grid[i][j].is_obstacle) {
                printf(ANSI_COLOR_WHITE "#" ANSI_COLOR_RESET);
            } else if (isCurrent) {
                printf(ANSI_COLOR_CYAN "*" ANSI_COLOR_RESET);
            } else if (isGhost) {
                printf(ANSI_COLOR_GRAY "*" ANSI_COLOR_RESET);
            } else {
                double hat = grid[i][j].hazard_arrival_time;
                if (hat <= current_time) { 
                     printf(ANSI_COLOR_RED "F" ANSI_COLOR_RESET);
                } else if (hat <= current_time + 10) { 
                     printf(ANSI_COLOR_YELLOW "~" ANSI_COLOR_RESET);
                } else { 
                     printf(ANSI_COLOR_GREEN "." ANSI_COLOR_RESET);
                }
            }
        }
        printf("\n");
    }
}

void log_metrics(char *run_type, double cpu_time, int path_length, bool success) {
    FILE *f = fopen("metrics.csv", "a");
    if (f == NULL) return;
    
    fseek(f, 0, SEEK_END);
    if (ftell(f) == 0) {
        fprintf(f, "RunType,CPUTime_ms,PathLength,Success\n");
    }
    fprintf(f, "%s,%.4f,%d,%s\n", run_type, cpu_time, path_length, success ? "TRUE" : "FALSE");
    fclose(f);
}

// ---------------------------------------------------------------------------
// 4. MinHeap for Dijkstra
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
// 5. BFS Algorithms and Logic
// ---------------------------------------------------------------------------

// Waterflow Validity Test (BFS 1)
void checkConnectivity(Point start, Point end) {
    if (grid[start.x][start.y].is_obstacle || grid[end.x][end.y].is_obstacle) {
        printf("FATAL: Map Unsolvable (Start/End Blocked)\n");
        exit(1);
    }

    bool visited[ROWS][COLS] = {false};
    Queue q;
    initQueue(&q);
    enqueue(&q, start);
    visited[start.x][start.y] = true;

    while (!isQueueEmpty(&q)) {
        Point curr = dequeue(&q);
        if (curr.x == end.x && curr.y == end.y) return; // Reachable

        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];
            if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS && 
                !grid[nx][ny].is_obstacle && !visited[nx][ny]) {
                visited[nx][ny] = true;
                Point next = {nx, ny};
                enqueue(&q, next);
            }
        }
    }
    
    printf("FATAL: Map Unsolvable\n");
    exit(1); 
}

// Hazard Propagation Engine (BFS 2)
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

        for (int i = 0; i < 4; i++) {
            int nx = curr.x + dx[i];
            int ny = curr.y + dy[i];

            if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS && !grid[nx][ny].is_obstacle) {
                if (grid[nx][ny].hazard_arrival_time == INF) {
                    grid[nx][ny].hazard_arrival_time = current_time + 1.0;
                    Point next = {nx, ny};
                    enqueue(&q, next);
                }
            }
        }
    }
}

PathNode* solve_dijkstra(Point start, Point end, double start_time_offset, double safety_buffer, char *run_name) {
    clock_t start_clock = clock();
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
    decreaseKey(minHeap, start.x, start.y, 0.0);

    // Clear console for animation start
    // printf("\033[2J"); // REMOVED: Managed by Main Loop

    bool success = false;
    double final_cost = 0;
    int path_len = 0;
    PathNode* pathHead = NULL;
    
    // Ghost path init
    ghostPathLen = 0;
    ghostFramesLeft = 0;

    while (!isHeapEmpty(minHeap)) {
        HeapNode u = extractMin(minHeap);
        int ux = u.x;
        int uy = u.y;
        double u_dist = u.dist;

        // --- ANIMATION START ---
        // Reconstruct path to current node for visualization
        Point tracePath[ROWS * COLS];
        int traceLen = 0;
        Point currTrace = {ux, uy};
        while (currTrace.x != -1) {
            tracePath[traceLen++] = currTrace;
            currTrace = parent[currTrace.x][currTrace.y];
        }
        
        // Render
        // render_map(tracePath, traceLen, start_time_offset + u_dist); 
        // sleep_ms(20); // REMOVED: Causing simulation to appear frozen due to 2500+ iterations
        
        // Update Ghost Path for NEXT frame
        // (Copy current trace to ghost path)
        for(int k=0; k<traceLen; k++) ghostPath[k] = tracePath[k];
        ghostPathLen = traceLen;
        ghostFramesLeft = 5; // Sustain for 5 frames (ticks) - logically here just next render iteration(s)
        // --- ANIMATION END ---

        if (ux == end.x && uy == end.y) {
            success = true;
            final_cost = u_dist;
            printf("\n[%s] Path found! Cost: %.2f\n", run_name, u_dist);
            
            // Reconstruct path as Linked List
            Point curr = end;
            while (curr.x != -1) {
                pushFront(&pathHead, curr.x, curr.y);
                grid[curr.x][curr.y].is_path = 1; 
                path_len++;
                curr = parent[curr.x][curr.y];
            }
            
            // Print path from list as verification
            PathNode* temp = pathHead;
            printf("Path List: ");
            while(temp) {
                printf("(%d,%d) -> ", temp->x, temp->y);
                temp = temp->next;
            }
            printf("END\n");

            free(minHeap->data);
            free(minHeap->pos);
            free(minHeap);
            goto end_dijkstra;
        }
        
        if (u_dist == INF) break;

        for (int i = 0; i < 4; i++) {
            int vx = ux + dx[i];
            int vy = uy + dy[i];

            if (vx >= 0 && vx < ROWS && vy >= 0 && vy < COLS && !grid[vx][vy].is_obstacle) {
                double weight = (double)grid[vx][vy].base_cost;
                double new_dist = u_dist + weight;

                // THE CHECK: Temporal Safety Buffer Novelty
                // Check if arrival time at neighbor (start_time + new_dist) allows for safety buffer
                if ((start_time_offset + new_dist + safety_buffer) < grid[vx][vy].hazard_arrival_time) {
                    if (new_dist < dist[vx][vy]) {
                        dist[vx][vy] = new_dist;
                        parent[vx][vy] = (Point){ux, uy};
                        decreaseKey(minHeap, vx, vy, new_dist);
                    }
                }
            }
        }
        
        // Decrement ghost frames if valid (simulated persistence)
        if (ghostFramesLeft > 0) ghostFramesLeft--;
    }

    printf("\n[%s] No path found.\n", run_name);
    free(minHeap->data);
    free(minHeap->pos);
    free(minHeap);

end_dijkstra:;
    clock_t end_clock = clock();
    double cpu_time = ((double) (end_clock - start_clock)) / CLOCKS_PER_SEC * 1000.0;
    log_metrics(run_name, cpu_time, path_len, success);
    return pathHead; 
}

// ---------------------------------------------------------------------------
// Main - Simulation Loop
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Main - Simulation Loop
// ---------------------------------------------------------------------------

int main() {
    srand(time(NULL));

    // 0. Clean Grid
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            grid[i][j].is_obstacle = 0; 
            grid[i][j].is_path = 0;
            grid[i][j].base_cost = 1;
        }
    }

    // 1. Setup Reliable Replan Scenario
    // Wall at Col 15. Range Row 5 to 45 (Blocking middle).
    // Gaps at Top (0-4) and Bottom (46-49).
    // Main Gap at Row 10 (which will inevitably ignite).
    
    // Build Wall
    for(int i=5; i<45; i++) {
        if(i != 10) grid[i][15].is_obstacle = 1; 
    }
    
    Point start = {10, 5};   
    Point end = {10, 25};    
    Point hazards[] = {{45, 45}}; 
    int hazardCount = 1;

    grid[start.x][start.y].is_obstacle = 0;
    grid[end.x][end.y].is_obstacle = 0;

    printf("Starting Temporal Disaster Navigator...\n");
    printf("Scenario: Wall at x=15. Gap at y=10. Hazard at (45,45).\n");
    
    // 2. Pre-Calc Hazard Times
    runHazardBFS(hazards, hazardCount);
    
    // 3. Connectivity Check
    checkConnectivity(start, end);

    // 4. Initial Plan
    double current_time = 0.0;
    double safety_buffer = 3.0;
    
    printf("\nGenerating Initial Plan...\n");
    PathNode* currentPlan = solve_dijkstra(start, end, current_time, safety_buffer, "Initial_Plan");
    
    if (!currentPlan) {
        printf("FATAL: No initial path found.\n");
        return 1;
    }

    // 5. Simulation Loop
    Point agentPos = start;
    int tick = 0;
    int replans = 0;
    int risks_avoided = 0;
    
    while (agentPos.x != end.x || agentPos.y != end.y) {
        printf("\033[2J"); // Clear Screen (Header handled in render_map)
        
        // --- Event: Force Hazard Update at Tick 5 ---
        if (tick == 5) {
             grid[10][15].hazard_arrival_time = 0; 
             grid[9][15].hazard_arrival_time = 0; 
             grid[11][15].hazard_arrival_time = 0;
             // Calculate how many nodes in OLD plan are now dangerous
             PathNode* chk = currentPlan;
             while(chk) {
                 if (grid[chk->x][chk->y].hazard_arrival_time < INF) risks_avoided++; 
                 chk = chk->next;
             }
        }

        // --- Controller: Safety Check & Replan ---
        bool pathValid = true;
        PathNode* temp = currentPlan;
        int steps_ahead = 0;
        
        while(temp) {
            double arrival_at_node = current_time + steps_ahead; 
            if (arrival_at_node + safety_buffer >= grid[temp->x][temp->y].hazard_arrival_time) {
                pathValid = false;
                break;
            }
            temp = temp->next;
            steps_ahead++;
        }
        
        if (!pathValid) {
            // Replan from CURRENT position
            if(currentPlan) freePath(currentPlan);
            currentPlan = solve_dijkstra(agentPos, end, current_time, safety_buffer, "Emergency_Replan");
            replans++;
            
            if (!currentPlan) {
                printf("FATAL: trapped! No escape path found.\n");
                return 1;
            }
        }
        
        // --- Render ---
        Point renderPath[ROWS*COLS];
        int rLen = 0;
        temp = currentPlan;
        while(temp) {
            renderPath[rLen++] = (Point){temp->x, temp->y};
            temp = temp->next;
        }
        
        render_map(renderPath, rLen, current_time, tick, replans, "PREDICTIVE", safety_buffer);
        
        if (tick == 5) printf(ANSI_COLOR_RED "\n[EVENT] SUDDEN ERUPTION! Main Gap Blocked!\n" ANSI_COLOR_RESET);
        if (!pathValid) printf(ANSI_COLOR_YELLOW "\n[CONTROLLER] Safety Violation! Rerouting...\n" ANSI_COLOR_RESET);
        
        sleep_ms(100); 

        // --- Move Agent ---
        if (currentPlan) {
            if (currentPlan->x == agentPos.x && currentPlan->y == agentPos.y) {
                 PathNode* old = currentPlan;
                 currentPlan = currentPlan->next;
                 free(old);
            }
            if (currentPlan) {
                agentPos.x = currentPlan->x;
                agentPos.y = currentPlan->y;
            }
        } else {
             break;
        }
        
        current_time += 1.0;
        tick++;
    }

    if (agentPos.x == end.x && agentPos.y == end.y) {
        printf("\n" ANSI_COLOR_GREEN "==========================================" ANSI_COLOR_RESET);
        printf("\n" ANSI_COLOR_GREEN "         MISSION SUCCESS                  " ANSI_COLOR_RESET);
        printf("\n" ANSI_COLOR_GREEN "==========================================" ANSI_COLOR_RESET);
        printf("\nTotal Ticks:      %d", tick);
        printf("\nReplans Triggered: %d", replans);
        printf("\nRisks Avoided:     %d (Projected collisions)", risks_avoided);
        printf("\nSafety Margin:     %.1fs", safety_buffer);
        printf("\n==========================================\n");
    } else {
        printf("\nFAILURE: Agent terminated.\n");
    }

    return 0;
}
