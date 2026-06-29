#include <iostream>
#include <cmath> // Required for abs()

struct Point { int x; int y; };
struct MazeSize { int row; int col; };

// --- CUSTOMIZE HERE ---
const Point goalCells[] = {{6, 6}, {6, 7}, {7, 6}, {7, 7}}; 
const int numGoalCells = sizeof(goalCells) / sizeof(Point);
MazeSize mySize = {14, 14};


void print_flood_maze(MazeSize size, const Point goals[], int goalCount) {
    for (int r = 0; r < size.row; ++r) {
        for (int c = 0; c < size.col; ++c) {
            
            // Find the minimum distance to any of the goal cells (Manhattan distance)
            int minDistance = 999; 
            for (int i = 0; i < goalCount; ++i) {
                int dist = std::abs(r - goals[i].x) + std::abs(c - goals[i].y);
                if (dist < minDistance) {
                    minDistance = dist;
                }
            }

            // Print the calculated distance
            std::cout << minDistance << "\t";
        }
        std::cout << "\n";
    }
}

int main() {
    print_flood_maze(mySize, goalCells, numGoalCells);
    return 0;
}