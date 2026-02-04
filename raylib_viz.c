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

const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 800;
const int CELL_SIZE = 15;

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

PathNode* solve_dijkstra(Point start, Point end, double start_time_offset, double safety_buffer, bool ignore_safety_buffer) {
    // Reset Viz only if it's the primary pass (safe buffer pass) to avoid clearing viz on fallback
    if (!ignore_safety_buffer) {
        for(int i=0; i<ROWS; i++) for(int j=0; j<COLS; j++) dijkstraVisited[i][j] = false;
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
    decreaseKey(minHeap, start.x, start.y, 0.0);

    PathNode* pathHead = NULL;
    
    while (!isHeapEmpty(minHeap)) {
        HeapNode u = extractMin(minHeap);
        int ux = u.x;
        int uy = u.y;
        double u_dist = u.dist;
        
        dijkstraVisited[ux][uy] = true; // Visited

        if (ux == end.x && uy == end.y) {
            // Reconstruct path
            Point curr = end;
            while (curr.x != -1) {
                pushFront(&pathHead, curr.x, curr.y);
                grid[curr.x][curr.y].is_path = 1; 
                curr = parent[curr.x][curr.y];
            }
            free(minHeap->data);
            free(minHeap->pos);
            free(minHeap);
            return pathHead; 
        }
        
        if (u_dist == INF) break;

        // 4-Way Neighbor Check
        for (int i = 0; i < 4; i++) {
            int vx = ux + dx[i];
            int vy = uy + dy[i];

            if (vx >= 0 && vx < ROWS && vy >= 0 && vy < COLS && !grid[vx][vy].is_obstacle) {
                double weight = (double)grid[vx][vy].base_cost;
                
                // Fallback Logic: Penalty for hazards
                if (ignore_safety_buffer) {
                     // If arrival time >= hazard time, add MASSIVE penalty but don't block
                     if (start_time_offset + u_dist + weight >= grid[vx][vy].hazard_arrival_time) {
                         weight += 1000.0; 
                     }
                     // If arrival time is safe-ish but buffer violation? Keep normal weight, Risk calc handles it
                }
                
                double new_dist = u_dist + weight;

                if (ignore_safety_buffer) {
                    // Always relax if ignore_safety_buffer is true (Cost Penalty logic handles preference)
                    if (new_dist < dist[vx][vy]) {
                        dist[vx][vy] = new_dist;
                        parent[vx][vy] = (Point){ux, uy};
                        decreaseKey(minHeap, vx, vy, new_dist);
                    }
                }
                else {
                    // Strict Safety Check
                    if ((start_time_offset + new_dist + safety_buffer) < grid[vx][vy].hazard_arrival_time) {
                        if (new_dist < dist[vx][vy]) {
                            dist[vx][vy] = new_dist;
                            parent[vx][vy] = (Point){ux, uy};
                            decreaseKey(minHeap, vx, vy, new_dist);
                        }
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

// ---------------------------------------------------------------------------
// 5. Drawing & Tier-1 Visuals (Waterflow)
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

void DrawDSAVisuals() {
    int startX = 850;
    int startY = 400;
    
    DrawText("DSA VISUALS", startX, startY, 20, DARKPURPLE);
    
    if (currentState == STATE_WATERFLOW) {
        DrawText("Structure: QUEUE (FIFO)", startX, startY + 30, 18, DARKBLUE);
        // Draw last 10 items in queue or front items
        // Since we implement a circular buffer wrapper or simple array, let's just show front -> front+5
        
        int boxSize = 30;
        int gap = 5;
        int count = 0;
        
        for (int i = waterflowQueue.front; i < waterflowQueue.rear; i++) {
            if (count > 8) break;
            
            int x = startX + count * (boxSize + gap);
            int y = startY + 60;
            
            DrawRectangle(x, y, boxSize, boxSize, SKYBLUE);
            DrawRectangleLines(x, y, boxSize, boxSize, BLUE);
            
            if (i == waterflowQueue.front) DrawText("H", x+8, y+8, 10, BLACK); // Head
            
            count++;
        }
        DrawText(TextFormat("Size: %d", waterflowQueue.rear - waterflowQueue.front), startX, startY + 100, 15, GRAY);
    }
    else if (currentState == STATE_SIMULATION) {
        DrawText("Structure: LINKED LIST", startX, startY + 30, 18, DARKBLUE);
        
        PathNode* traverser = currentPlan;
        int boxSize = 30;
        int gap = 15; // larger gap for arrows
        int count = 0;
        
        while(traverser && count < 6) {
             int x = startX + count * (boxSize + gap);
             int y = startY + 60;
             
             DrawRectangle(x, y, boxSize, boxSize, GREEN);
             DrawRectangleLines(x, y, boxSize, boxSize, DARKGREEN);
             
             // Draw Arrow
             if (traverser->next) {
                 DrawLine(x + boxSize, y + boxSize/2, x + boxSize + gap, y + boxSize/2, BLACK);
             }
             
             traverser = traverser->next;
             count++;
        }
        if (traverser) DrawText("...", startX + count * (boxSize + gap), startY + 70, 20, BLACK);
    }
}


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

void DrawWaterflowRender() {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            int posX = j * CELL_SIZE + 50;
            int posY = i * CELL_SIZE + 50;
            
            if (grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, BLACK);
            } else if (waterflowVisited[i][j]) {
                // Water effect: Light Blue background + Blue dot
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, Fade(SKYBLUE, 0.2f));
                DrawCircle(posX + CELL_SIZE/2, posY + CELL_SIZE/2, 2, BLUE);
            } else {
                DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, Fade(LIGHTGRAY, 0.3f));
            }
        }
    }
    
    DrawText("PRE-FLIGHT CHECK: FLOOD FILL", 850, 50, 20, BLUE);
    if(waterflowComplete) {
        if (mapIsSolvable) {
             DrawText("REACHABILITY: CONFIRMED", 850, 100, 20, GREEN);
             DrawText("PRESS [SPACE] TO START", 850, 150, 20, BLACK);
        } else {
             DrawText("REACHABILITY: IMPOSSIBLE", 850, 100, 20, RED);
             DrawText("FATAL ERROR", 850, 150, 20, RED);
        }
    } else {
        DrawText("ANALYZING...", 850, 100, 20, ORANGE);
    }
}

void DrawMap(double current_time) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            int posX = j * CELL_SIZE + 50; 
            int posY = i * CELL_SIZE + 50;
            
            // Draw Cell Base
            if (grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, BLACK);
            } else {
                 DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, LIGHTGRAY);
            }
            
            // Draw Frontier (Yellow) - Low opacity to not obscure grid lines too much
            if (dijkstraVisited[i][j] && !grid[i][j].is_path && !grid[i][j].is_obstacle) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, Fade(YELLOW, 0.3f));
            }

            // Draw Hazards
            if (!grid[i][j].is_obstacle) {
                 if (grid[i][j].hazard_arrival_time <= current_time) {
                    DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, RED);
                 } else if (grid[i][j].hazard_arrival_time < INF && grid[i][j].hazard_arrival_time > current_time) {
                     // Future Hazard Visualization
                     DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, Fade(RED, 0.15f));
                 }
            }
            
            // Draw Path
            if (grid[i][j].is_path && grid[i][j].hazard_arrival_time > current_time) {
                DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, LIME);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Temporal Disaster Navigator");
    SetTargetFPS(60); 

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
            
            // On First Frame of Sim, Plan
            if (currentPlan == NULL && tick == 0) {
                 // Try Safe Path
                 currentPlan = solve_dijkstra(startVal, endVal, current_time, SAFETY_BUFFER, false);
                 if (currentPlan) {
                     currentRisk = 0.0;
                 } else {
                     // Fallback: Risky Path
                     currentPlan = solve_dijkstra(startVal, endVal, current_time, SAFETY_BUFFER, true);
                     if (currentPlan) currentRisk = CalculateRisk(currentPlan, current_time);
                     else {
                         currentRisk = INF;
                         // Retry planning every 60 frames if start was blocked, just in case? 
                         // For now, just let it be NULL.
                     }
                 }
            }

            frameCounter++;
            // Simulation Speed: Update every 10 frames
            if (frameCounter >= 10 && agentPos.x != -1 && currentState != STATE_GAMEOVER) { 
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
                 
                 // Also re-check risk buffer to see if we should replan for a safer path if one opened up?
                 // For now, let's keep it simple: Replan if STRICTLY blocking.
                 
                 if (!pathValid) {
                     if(currentPlan) freePath(currentPlan);
                     
                     // Try Safe
                     currentPlan = solve_dijkstra(agentPos, endVal, current_time, SAFETY_BUFFER, false);
                     if (currentPlan) {
                         currentRisk = 0.0;
                     } else {
                         // Fallback
                         currentPlan = solve_dijkstra(agentPos, endVal, current_time, SAFETY_BUFFER, true);
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
            ClearBackground(RAYWHITE);
            
            if (currentState == STATE_SETUP) {
                // Draw Grid in Setup Mode
                for (int i = 0; i < ROWS; i++) {
                    for (int j = 0; j < COLS; j++) {
                         int posX = j * CELL_SIZE + 50; 
                         int posY = i * CELL_SIZE + 50;
                         if (grid[i][j].is_obstacle) DrawRectangle(posX, posY, CELL_SIZE, CELL_SIZE, BLACK);
                         else DrawRectangleLines(posX, posY, CELL_SIZE, CELL_SIZE, LIGHTGRAY);
                         
                         // Show Hazards
                         for(int k=0; k<hazardCount; k++) {
                             if(hazards[k].x == i && hazards[k].y == j) DrawCircle(posX + CELL_SIZE/2, posY + CELL_SIZE/2, CELL_SIZE/3, RED);
                         }
                    }
                }
                
                // Agents
                DrawRectangle(startVal.y * CELL_SIZE + 50, startVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, BLUE);
                DrawRectangle(endVal.y * CELL_SIZE + 50, endVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, GREEN);
                
                DrawText("SETUP MODE", 850, 50, 30, DARKGRAY);
                DrawText("[ENTER] RUN SIMULATION", 850, 100, 20, BLACK);
                
                DrawText("INPUT MODE:", 850, 150, 20, DARKGRAY);
                Color c1 = currentInputMode==INPUT_WALL? RED:GRAY;
                Color c2 = currentInputMode==INPUT_HAZARD? RED:GRAY;
                Color c3 = currentInputMode==INPUT_START? RED:GRAY;
                Color c4 = currentInputMode==INPUT_END? RED:GRAY;
                Color c5 = currentInputMode==INPUT_ERASER? RED:GRAY;
                
                DrawText("[1] WALL", 850, 180, 20, c1);
                DrawText("[2] HAZARD", 850, 205, 20, c2);
                DrawText("[3] START", 850, 230, 20, c3);
                DrawText("[4] END", 850, 255, 20, c4);
                DrawText("[5] ERASER", 850, 280, 20, c5);
                
                DrawText("Mouse Left: Place/Action | Right: Clear", 850, 310, 15, DARKGRAY);
            }
            else if (currentState == STATE_WATERFLOW) {
                DrawWaterflowRender();
                // Draw Start/End for context
                DrawRectangle(startVal.y * CELL_SIZE + 50, startVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(BLUE, 0.5f));
                DrawRectangle(endVal.y * CELL_SIZE + 50, endVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(GREEN, 0.5f));
                
                DrawDSAVisuals();
            }
            else {
                // Sim Mode
                DrawMap(current_time);
                
                DrawRectangle(agentPos.y * CELL_SIZE + 50, agentPos.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, BLUE);
                 // Draw Exit Check
                DrawRectangle(endVal.y * CELL_SIZE + 50, endVal.x * CELL_SIZE + 50, CELL_SIZE, CELL_SIZE, Fade(GREEN, 0.3f));

                // Dashboard
    DrawText(TextFormat("TIME: %.1f", current_time), 850, 50, 20, BLACK);
    
    Color statusColor = GREEN;
    char* statusText = "NORMAL";
    if (tick > 5 && tick < 12 && replans > 0) { statusColor = RED; statusText = "CRITICAL / REPLAN"; }
    
    DrawText(TextFormat("STATUS: %s", statusText), 850, 80, 20, statusColor);
    DrawText(TextFormat("REPLANS: %d", replans), 850, 110, 20, BLACK);
    DrawText(TextFormat("EVENT: %s", eventMessage), 850, 140, 20, DARKPURPLE);
    
    // Risk Display
    Color riskColor = GREEN;
    if (currentRisk > 5.0) riskColor = ORANGE;
    if (currentRisk > 20.0) riskColor = RED;
    if (currentPlan == NULL) {
        DrawText("NO PATH FOUND!", 850, 170, 20, RED);
    } else {
        if (currentRisk >= DBL_MAX || currentRisk > 100000) {
             DrawText("RISK: EXTREME", 850, 170, 20, RED);
        } else {
             DrawText(TextFormat("RISK: %.1f", currentRisk), 850, 170, 20, riskColor);
        }
        if (currentRisk > 0 && currentRisk < 100000) DrawText("(Buffer Violation)", 960, 175, 10, RED);
    }
    
    DrawText("LEGEND:", 850, 300, 20, DARKGRAY);
    DrawText("YELLOW: Search Frontier", 850, 330, 15, YELLOW);
    DrawText("LIME: Agent Path (Safe)", 850, 350, 15, LIME);
    DrawText("RED (Faint): Future Heat", 850, 370, 15, Fade(RED, 0.5f));
    
    if (currentState == STATE_GAMEOVER) {
                    DrawText("MISSION SUCCESS", 850, 200, 30, GREEN);
                    DrawText("PRESS [R] TO RESTART", 850, 240, 20, DARKGRAY);
                }
                
                DrawDSAVisuals();
            }

        EndDrawing();
    }

    return 0;
}

