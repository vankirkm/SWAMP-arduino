#include <string.h>
#include <float.h>

#define MAP_SIZE 20
#define PATH_LENGTH 100

struct Node {
    Node* parent;
    byte row;
    byte col;
    byte cost;
    byte direction;
    bool obstacle;
    bool visited;
};

class PathFinder {

public:

    void findPath(byte navGrid[MAP_SIZE][MAP_SIZE], byte pathList[], int row, int col, Node goal) {

        Node frontier[PATH_LENGTH];
        int first = 0;
        int last = 0;
        Node navGridNodes[MAP_SIZE][MAP_SIZE];

        for(int i = 0; i < MAP_SIZE; i++) {
            for(int j = 0; j < MAP_SIZE; j++) {
                bool obstacle = navGrid[i][j] == 1;
                navGridNodes[i][j] = createNewNode(i, j, goal, obstacle);
            }
        }

        // initialize the queue and the first node
        frontier[0] = navGridNodes[row][col];
        Node current;
        Node neighbors[8];
        int numNeighbors = 0;
        current = frontier[first];
        current.parent = NULL;

        while(current.row != goal.row && current.col != goal.col) {
            current.visited = true;
            current = frontier[first];
            first = (1 + first) % PATH_LENGTH;
            //add current node's 8 neighbors to the queue
            if(current.row - 1 >= 0) {
                if(current.col - 1 >= 0) {
                    addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col - 1][current.row - 1], goal, 1);
                }
                if(current.col + 1 < MAP_SIZE) {
                    addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col + 1][current.row - 1], goal, 3);
                }
                addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col][current.row - 1], goal, 2);
            }
            if(current.col - 1 >= 0) {
                addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col - 1][current.row], goal, 4);

                if(current.row + 1 < MAP_SIZE) {
                    addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col - 1][current.row + 1], goal, 6);
                }
            }
            if(current.col + 1 < MAP_SIZE) {
                addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col + 1][current.row], goal, 5);

                if(current.row + 1 < MAP_SIZE) {
                    addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col + 1][current.row + 1], goal, 8);
                }
            }

            if(current.row + 1 < MAP_SIZE) {
                addToQueue(neighbors, numNeighbors, current, navGridNodes[current.col][current.row + 1], goal, 7);
            }
            for(int i = 0; i < numNeighbors; i++) {
                frontier[last + i] = neighbors[i];
                last = (1 + last) % PATH_LENGTH;
            }
            current = frontier[first];
        }

        getPathList(pathList, current);
    }

private:

    // 8 directions encoded as bytes arranged as follows:
    //       1(NW) 2(N) 3(NE)
    //       4(W)       5(E)
    //       6(SW) 7(S) 8(SE)
    void getPathList(byte pathList[], Node current) {
        Node prev = current;
        int index = 0;
        while(current.parent != NULL) {
            pathList[index] = current.direction;
            index++;
            current = *current.parent;
        }
    }

    Node createNewNode(int row, int col, Node goal, bool obstacle) {
        Node newNode;
        newNode.col = row;
        newNode.row = col;
        newNode.visited = false;
        newNode.obstacle = obstacle;
        return newNode;
    }

    // this is bad. we should be using a priority queue for this, but timeline
    // and memory requirements say otherwise
    void addToQueue(Node neighbors[8], int& numNeighbors, Node current, Node node, Node goal, byte direction) {
        // only add a node back to the queue if we haven't already
        if(node.visited == false) {
            node.parent = &current;
            numNeighbors++;
            node.cost = calculateCost(node, goal) + current.cost;
            for(int i = 0; i <= numNeighbors; i++) {
                if(neighbors[i].cost > node.cost) {
                    Node temp = neighbors[i];
                    Node temp2;
                    neighbors[i] = node;
                    for(int j = i + 1; j < numNeighbors; j++) {
                        temp2 = neighbors[j];
                        neighbors[j] = temp;
                        temp = temp2;
                    }
                }
            }
        }
    }

    double calculateCost(Node current, Node goal) {
        if(current.obstacle)
            return DBL_MAX;
        // euclidean distance
        return sqrt(pow((current.row - goal.row), 2) + pow((current.col - goal.col), 2));
    }

};


