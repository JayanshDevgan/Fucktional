#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <GLM/glm.hpp>
#include <limits>
#include <queue>
#include <algorithm>

/* When shit hits the fan in a wild-ass orgy of chaos, grab the number of threads and run at least 5 paths on each one (SYSTEM DEPENDENT, partner).
WARNING: Ain’t got a damn clue if this’ll hold up in the main job as planned. */

// --- Forward Declarations ---
class LinkedList;
struct DijkstraNode;

// --- Node Class ---
class Node {
private:
    std::vector<glm::vec2> m_path; // The route this node's holdin', like a map to the score
    Node* next;                    // Pointer to the next stop in this wild ride

public:
    Node(const std::vector<glm::vec2>& path) : m_path(path), next(nullptr) {} // Build a new node with the path, no connections yet
    ~Node() { /* Ain't cleanin' up the whole chain here, partner */ }          // Destructor, but we're keepin' it simple

    const std::vector<glm::vec2>& getPath() const { return m_path; }          // Get the path, read-only, like a blueprint you can't touch

    std::vector<glm::vec2>& getPathMutable() { return m_path; }              // Get the path you can mess with, like a plan you're redrawin'

    void setNext(Node* newNext) { next = newNext; }                          // Hook up the next node in the chain, like settin' up the getaway
    Node* getNext() const { return next; }                                   // Check the next stop, no messin' around
};

// --- LinkedList Class ---
class LinkedList {
private:
    Node* head;        // The start of this crazy road trip
    size_t listSize;   // How many paths we're haulin' in this rig

public:
    LinkedList() : head(nullptr), listSize(0) {} // Kick off with an empty crew, no paths, no problems

    ~LinkedList() { // Clean up the whole damn convoy
        Node* current = head;
        while (current != nullptr) {
            Node* nextNode = current->getNext();
            delete current; // Blow up the current node, no evidence left
            current = nextNode;
        }
        head = nullptr; // Wipe the slate clean
    }

    void append(const std::vector<glm::vec2>& path) { // Add a new path to the crew's list
        Node* newNode = new Node(path);

        if (head == nullptr) { // If the road's empty, this is the first ride
            head = newNode;
        }
        else { // Otherwise, chase down the last node and hook it up
            Node* current = head;
            while (current->getNext() != nullptr) {
                current = current->getNext();
            }
            current->setNext(newNode);
        }
        listSize++; // Bump up the tally of paths we're runnin'
    }

    void printList() const { // Lay out the whole damn map for the crew
        std::cout << "Linked List Contents (" << listSize << " paths):\n";
        Node* current = head;
        int index = 0;

        if (head == nullptr) { // If the road's deserted
            std::cout << "  (List is empty, nothin' to see here)\n";
            return;
        }

        while (current != nullptr) { // Run through each path like checkin' the loot
            const std::vector<glm::vec2>& path = current->getPath();
            std::cout << "Path " << index << " (Length: " << path.size() << "): ";
            for (const auto& pos : path) {
                std::cout << "(" << std::fixed << std::setprecision(2) << pos.x << ", " << pos.y << ") ";
            }
            std::cout << "\n";
            current = current->getNext();
            index++;
        }
    }

    size_t getSize() const { return listSize; } // How many jobs we got lined up?

    // Lay out a single path like a heist plan on a chalkboard
    void printPathAsTree(const std::vector<glm::vec2>& path, const std::vector<glm::vec2>& allNodes, bool isShortest = false) const {
        if (path.empty()) { // No path? Ain't plannin' a job with no map
            std::cout << "  (Path is empty, boss)\n";
            return;
        }

        std::cout << (isShortest ? "--- SHORTEST PATH, THE CLEAN GETAWAY ---\n" : "--- Path Laid Out, Like a Smuggler's Route ---\n");
        for (size_t i = 0; i < path.size(); ++i) {
            long original_node_idx = -1;
            for (size_t k = 0; k < allNodes.size(); ++k) {
                // Check if this spot's close enough to count
                if (glm::distance(allNodes[k], path[i]) < 0.001f) {
                    original_node_idx = k;
                    break;
                }
            }

            for (size_t j = 0; j < i; ++j) {
                // Draw the lines like a roadmap, keep it tight
                bool hasMoreNodesBelow = (i < path.size() - 1);
                if (hasMoreNodesBelow) {
                    std::cout << "|   "; // Keep the trail marked
                }
                else {
                    std::cout << "    "; // Clear the way
                }
            }

            if (i == 0) { // Start of the job
                std::cout << "|--- START: (" << std::fixed << std::setprecision(2) << path[i].x << ", " << path[i].y << ") [Node " << original_node_idx << "]\n";
            }
            else if (i == path.size() - 1) { // End of the line
                std::cout << "'--- END:   (" << std::fixed << std::setprecision(2) << path[i].x << ", " << path[i].y << ") [Node " << original_node_idx << "]\n";
            }
            else { // Checkpoints along the way
                std::cout << "--> NODE:  (" << std::fixed << std::setprecision(2) << path[i].x << ", " << path[i].y << ") [Node " << original_node_idx << "]\n";
            }
        }
        std::cout << "------------------------------------------\n";
    }

    // Iterators to roll through the paths
    class Iterator {
    private:
        Node* current; // Where we at in this job
    public:
        Iterator(Node* node) : current(node) {}

        // Get the path you can rewrite, like changin' the plan
        std::vector<glm::vec2>& operator*() { return current->getPathMutable(); }

        // Get the path for lookin' only, no touchin'
        const std::vector<glm::vec2>& operator*() const { return current->getPathMutable(); }

        Iterator& operator++() { current = current->getNext(); return *this; } // Move to the next job
        bool operator!=(const Iterator& other) const { return current != other.current; } // Check if we're at the same spot
    };

    Iterator begin() { return Iterator(head); } // Start of the heist
    Iterator end() { return Iterator(nullptr); } // End of the road
    const Iterator begin() const { return Iterator(head); } // Start, no changin' nothin'
    const Iterator end() const { return Iterator(nullptr); } // End, keep it locked

    const std::vector<glm::vec2>* getFirstPath() const { // Grab the first path in the stash
        if (head != nullptr) {
            return &(head->getPath());
        }
        return nullptr; // No paths, no dice
    }
};

// --- Utility Functions ---
float randomFloat(float min, float max) { // Roll the dice for a random number
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen); // Spit out the result
}

glm::vec2 randomVec2(float min, float max) { // Cook up a random 2D spot
    return glm::vec2(randomFloat(min, max), randomFloat(min, max));
}

void GenerateRandomVectorList(int n, float min, float max, std::vector<glm::vec2>& positionList) { // Build a list of random hideouts
    for (int i = 0; i < n; ++i) {
        positionList.push_back(randomVec2(min, max));
    }
}

void printVector(const std::vector<glm::vec2>& vec) { // Show off the coordinates like a wanted poster
    std::cout << "Vector Contents (" << vec.size() << " elements):\n";
    std::cout << std::fixed << std::setprecision(2);

    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << "[" << i << "] ("
            << vec[i].x << ", "
            << vec[i].y << ")\n";
    }
}

float calculateDistance(const glm::vec2& a, const glm::vec2& b) { // How far between two spots?
    return glm::length(a - b); // Straight-up distance, no bullshit
}

// Total up the distance of a path, like countin' the miles on a job
float calculatePathLength(const std::vector<glm::vec2>& path) {
    float totalLength = 0.0f;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        totalLength += calculateDistance(path[i], path[i + 1]);
    }
    return totalLength; // Total haul in distance
}

size_t findNearestNode(const std::vector<glm::vec2>& nodes, size_t currentIndex) { // Find the closest hideout to the current spot
    if (nodes.empty()) return (size_t)-1; // No nodes? Get outta here

    const glm::vec2& currentPos = nodes[currentIndex];
    size_t nearestIndex = (size_t)-1;
    float nearestDistance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i != currentIndex) { // Don't check the spot you're already at
            float distance = calculateDistance(currentPos, nodes[i]);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestIndex = i; // Mark the closest joint
            }
        }
    }
    return nearestIndex; // Return the index of the nearest hideout
}

// --- Dijkstra's Algorithm Implementation ---
struct DijkstraNode {
    size_t index;                      // Which spot we're at
    float distance;                    // How far we've traveled
    std::vector<glm::vec2> path;       // The route we took to get here

    bool operator>(const DijkstraNode& other) const { // Compare which path's shorter
        return distance > other.distance;
    }
};

void findShortestPathToClosest(const std::vector<glm::vec2>& nodes, size_t startIndex, const glm::vec2& targetCoordinates, LinkedList& shortestPathList) {
    if (nodes.empty()) { // No nodes? Job's a bust
        std::cout << "Nodes list is empty. Can't plan a route.\n";
        return;
    }
    if (startIndex >= nodes.size()) { // Startin' point's outta bounds? Trouble
        std::cout << "Start index is off the map.\n";
        return;
    }

    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq; // Priority queue for the shortest route
    std::vector<float> dist(nodes.size(), std::numeric_limits<float>::max()); // Track distances to each spot
    std::vector<std::vector<glm::vec2>> shortestPathsToNode(nodes.size()); // Store the best paths

    dist[startIndex] = 0.0f; // Startin' point's got no distance yet
    shortestPathsToNode[startIndex].push_back(nodes[startIndex]); // Mark the start
    pq.push({ startIndex, 0.0f, {nodes[startIndex]} }); // Load up the first job

    std::cout << "\nRunnin' Dijkstra's from Node " << startIndex << " to the closest spot to target ("
        << std::fixed << std::setprecision(2) << targetCoordinates.x << ", "
        << targetCoordinates.y << ")...\n";

    float minDistanceToTargetOverall = std::numeric_limits<float>::max(); // Closest we've gotten to the target
    size_t closestNodeIndex = (size_t)-1; // Index of the nearest hideout
    std::vector<glm::vec2> finalShortestPathToClosest; // The winning route

    while (!pq.empty()) { // Keep rollin' through the queue
        DijkstraNode current = pq.top();
        pq.pop();

        size_t u_index = current.index;
        float u_distance = current.distance;
        std::vector<glm::vec2> currentPath = current.path;

        if (u_distance > dist[u_index]) { // If we found a better route already, skip
            continue;
        }

        float distanceToTargetFromCurrentNode = calculateDistance(nodes[u_index], targetCoordinates); // How close are we to the prize?
        if (distanceToTargetFromCurrentNode < minDistanceToTargetOverall) { // New best route?
            minDistanceToTargetOverall = distanceToTargetFromCurrentNode;
            closestNodeIndex = u_index;
            finalShortestPathToClosest = currentPath;
            std::cout << "  New closest hideout found: Node " << u_index
                << " (dist to target: " << std::fixed << std::setprecision(2) << minDistanceToTargetOverall << ")\n";
        }

        for (size_t v_index = 0; v_index < nodes.size(); ++v_index) { // Check all possible next stops
            if (u_index == v_index) continue; // Don't double back

            float weight = calculateDistance(nodes[u_index], nodes[v_index]); // Distance to the next spot
            if (dist[u_index] + weight < dist[v_index]) { // Found a shorter route?
                dist[v_index] = dist[u_index] + weight;
                std::vector<glm::vec2> newPath = currentPath;
                newPath.push_back(nodes[v_index]); // Add the next stop
                shortestPathsToNode[v_index] = newPath;
                pq.push({ v_index, dist[v_index], newPath }); // Queue up the new route
            }
        }
    }

    if (closestNodeIndex != (size_t)-1) { // If we found a winner
        std::cout << "\nLocked in the shortest path to the closest node: Node " << closestNodeIndex
            << " (distance to target: " << std::fixed << std::setprecision(2) << minDistanceToTargetOverall << ")\n";
        shortestPathList.append(finalShortestPathToClosest); // Store the golden route
    }
    else {
        std::cout << "Dijkstra's: Couldn't find a damn thing to reach.\n"; // No dice
    }
}

// --- Recursive helper for findin' ALL paths with limits ---
void findAllPathsRecursive(
    const std::vector<glm::vec2>& nodes,
    const glm::vec2& targetCoordinates,
    size_t currentIndex,
    std::vector<glm::vec2>& currentPath,
    std::vector<bool>& visited,
    std::vector<std::vector<glm::vec2>>& foundPathsContainer, // Stash for all the routes we find
    float targetReachTolerance,
    size_t maxPathLength,
    size_t maxPathsToFind)
{
    // Add the current spot to the route and mark it as hit
    currentPath.push_back(nodes[currentIndex]);
    visited[currentIndex] = true;

    // Check if we're close enough to the target to call it a score
    if (calculateDistance(nodes[currentIndex], targetCoordinates) < targetReachTolerance) {
        foundPathsContainer.push_back(currentPath); // Bank the route
        // If we've got enough routes, we're done here
        if (foundPathsContainer.size() >= maxPathsToFind) {
            // This stops us from goin' overboard, but it ain't perfect
        }
    }

    // Bail if the route's too long or we've got enough
    if (currentPath.size() >= maxPathLength || foundPathsContainer.size() >= maxPathsToFind) {
        currentPath.pop_back(); // Back it up
        visited[currentIndex] = false; // Clear the mark
        return;
    }

    // Scout out the next possible stops
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!visited[i]) { // Only hit unvisited spots
            findAllPathsRecursive(nodes, targetCoordinates, i, currentPath, visited,
                foundPathsContainer, targetReachTolerance, maxPathLength, maxPathsToFind);
            // If we've got enough routes, cut and run
            if (foundPathsContainer.size() >= maxPathsToFind) {
                break;
            }
        }
    }

    // Backtrack, like slippin' outta a bad deal
    currentPath.pop_back();
    visited[currentIndex] = false;
}

// --- Wrapper to kick off the path hunt ---
void findAllPathsLimited(
    const std::vector<glm::vec2>& nodes,
    const glm::vec2& targetCoordinates,
    size_t startIndex,
    LinkedList& allPathsContainer,
    float targetReachTolerance,
    size_t maxPathLength,
    size_t maxPathsToFind)
{
    std::vector<std::vector<glm::vec2>> tempFoundPaths; // Temporary stash for routes
    std::vector<glm::vec2> currentPath; // The route we're buildin'
    std::vector<bool> visited(nodes.size(), false); // Who's been hit?

    std::cout << "\nGunnin' for up to " << maxPathsToFind
        << " paths (max length " << maxPathLength << ") from Node " << startIndex
        << " to within " << targetReachTolerance << " units of the target...\n";

    findAllPathsRecursive(nodes, targetCoordinates, startIndex, currentPath, visited,
        tempFoundPaths, targetReachTolerance, maxPathLength, maxPathsToFind);

    // Load the found routes into the main stash
    for (const auto& path : tempFoundPaths) {
        allPathsContainer.append(path);
    }
    std::cout << "Bagged " << tempFoundPaths.size() << " paths within the limits.\n";
}

// --- Main Function ---
int main() {
    int numVector = 250; // Number of hideouts to generate
    float minValue = -10.0f; // Lowest coordinate for the map
    float maxValue = 50.0f; // Highest coordinate for the map

    std::vector<glm::vec2> positionNode; // The list of spots we're workin' with
    GenerateRandomVectorList(numVector, minValue, maxValue, positionNode); // Cook up some random hideouts

    glm::vec2 target = randomVec2(minValue, maxValue); // Pick a random target spot

    std::cout << "Generated Target Position: (" << std::fixed << std::setprecision(2) << target.x << ", " << target.y << ")" << std::endl;

    printVector(positionNode); // Show the map

    size_t nearestIndex = findNearestNode(positionNode, 0); // Find the closest spot to Node 0
    std::cout << "\nNearest node to Node 0 (excluding itself) is Node " << nearestIndex
        << " at position (" << std::fixed << std::setprecision(2) << positionNode[nearestIndex].x
        << ", " << positionNode[nearestIndex].y << ")\n";

    // --- Part 1: Find the shortest path using Dijkstra's ---
    LinkedList shortestPathByDijkstra; // Stash for the cleanest getaway
    size_t startNodeIndex = 0; // Where we're kickin' off
    findShortestPathToClosest(positionNode, startNodeIndex, target, shortestPathByDijkstra); // Run the job

    // --- Part 2: Find all (limited) paths using DFS ---
    LinkedList allLimitedPaths; // Stash for all the routes we find
    // Parameters for the path hunt:
    float targetReachToleranceForDFS = 15.0f; // How close we gotta get to the target
    size_t maxPathLength = 5;                // Max stops in a route
    size_t maxPathsToFind = 10;               // Max number of routes to bag

    findAllPathsLimited(positionNode, target, startNodeIndex, allLimitedPaths,
        targetReachToleranceForDFS, maxPathLength, maxPathsToFind); // Hunt for all routes

    // --- Compare and Print Results ---

    std::cout << "\n=== Layin' Out the Scores ===\n";

    // 1. Show all the routes we found with DFS
    if (allLimitedPaths.getSize() > 0) {
        std::cout << "\n--- All Paths Found (Limited DFS, Ready to Roll) ---\n";
        int path_count = 0;
        for (const auto& path : allLimitedPaths) { // Roll through each route
            std::cout << "\nPath " << ++path_count << ": (Length: " << calculatePathLength(path) << ")\n";
            allLimitedPaths.printPathAsTree(path, positionNode); // Lay it out like a plan
        }
    }
    else {
        std::cout << "\nNo paths found by limited DFS. Bad intel.\n"; // No routes, no glory
    }

    // 2. Show the shortest path from Dijkstra's
    std::cout << "\n--- Shortest Path (Dijkstra's Clean Getaway) ---\n";
    const std::vector<glm::vec2>* dijkstraPath = shortestPathByDijkstra.getFirstPath();
    if (dijkstraPath) { // If we got a winner
        std::cout << "Dijkstra's Path Total Length: " << std::fixed << std::setprecision(2) << calculatePathLength(*dijkstraPath) << "\n";
        shortestPathByDijkstra.printPathAsTree(*dijkstraPath, positionNode, true); // Show it off, marked as the best
    }
    else {
        std::cout << "Dijkstra's didn't find a damn path.\n"; // Busted
    }

    // 3. Find the shortest route among the DFS paths for comparison
    if (allLimitedPaths.getSize() > 0) {
        float trueShortestLengthAmongDFS = std::numeric_limits<float>::max(); // Best distance so far
        const std::vector<glm::vec2>* trueShortestDFSPath = nullptr; // Best route from DFS

        for (const auto& path : allLimitedPaths) { // Check each DFS route
            float currentPathLength = calculatePathLength(path);
            if (currentPathLength < trueShortestLengthAmongDFS) { // New best?
                trueShortestLengthAmongDFS = currentPathLength;
                trueShortestDFSPath = &path; // Mark it
            }
        }

        std::cout << "\n--- Shortest Path Among Limited DFS Routes ---\n";
        if (trueShortestDFSPath) { // If we found one
            std::cout << "Shortest DFS Path Total Length: " << std::fixed << std::setprecision(2) << trueShortestLengthAmongDFS << "\n";
            allLimitedPaths.printPathAsTree(*trueShortestDFSPath, positionNode); // Show the best DFS route
        }
    }

    return 0; // Job's done, time to ride out
}