# Fucktional Pathfinding Algorithm: A Visual Guide to Finding Paths

## Welcome!

This project, playfully named "Fucktional" (because it's functional and robust!), is a C++ program designed to help you understand how computers find the best way to get from one point to another. Imagine you have a map with many locations, and you want to find the shortest route to a specific destination. This program does exactly that, but in a simple 2D world, and it even shows you how it works step-by-step!

## What Problem Does This Solve?

At its heart, this program tackles the "pathfinding problem." This is a fundamental challenge in many areas, like:

* **GPS navigation:** Finding the quickest route on a map.

* **Video games:** Making characters move intelligently through a game world.

* **Robotics:** Planning how a robot should move to avoid obstacles.

* **Network routing:** Deciding the best way for data to travel across the internet.

Our program focuses on finding paths between randomly placed "locations" (nodes) to a specific "destination" (target) in a simple 2D grid.

## How Does It Work? (The Core Algorithms)

First, we need a map! In our program, the "map" is a collection of `N` random 2D points. We call these points **"Nodes."** Each node has an X and a Y coordinate (like `(10.5, 23.1)`).

* **Implicit Connections:** Unlike a real map with roads, our nodes are "connected" to *every other node*. The "distance" or "cost" to travel between any two nodes is simply the straight-line distance between them (we call this **Euclidean Distance**).

### Dijkstra's Algorithm (The "Shortest Path Finder")

Dijkstra's (pronounced "Dike-stra's") is a famous algorithm that guarantees finding the **shortest path** from one starting node to all other nodes in a graph.

* **Our Goal:** In this program, we use Dijkstra's to find the shortest path from a designated start node to the node among the generated set that is geometrically closest to the target coordinate. The `Target` itself isn't necessarily one of our nodes; it's just a point we want to get close to.

* **How it Works (Simply):** Imagine Dijkstra's as a smart explorer. It starts at your beginning point and gradually "discovers" the shortest way to reach every other point. It always prioritizes exploring paths that seem shortest so far, ensuring it never misses a shorter route. It keeps track of the shortest distance found to each node and the path taken to get there.

### Depth-First Search (DFS) (The "Path Explorer")

While Dijkstra's is great for finding *the* shortest path, sometimes you want to see *other* possible paths, even if they're not the absolute shortest. That's where Depth-First Search comes in.

* **How it Works (Simply):** DFS is like a curious explorer who goes as deep as possible down one path before backtracking and trying another. It explores one branch completely before moving to the next.

* **Why "Limited" Here?:** If we let DFS run wild on our map of 25 nodes (where every node connects to every other node), it would find an astronomical number of paths – far too many to display or even compute! To make it practical for demonstration, we put **limits** on DFS:

  * **Maximum Path Length:** We only look for paths that are a certain number of nodes long (e.g., paths with 5 steps or less).

  * **Maximum Paths to Find:** We stop collecting paths after we've found a certain number of them (e.g., the first 10 paths).

* **Its Purpose:** This limited DFS helps us generate a *sample* of different paths, which we can then compare to the single, optimal path found by Dijkstra's.

## Key Concepts for Beginners

* **Node:** A single point or location in our 2D space, represented by `(X, Y)` coordinates.

* **Path:** A sequence of connected nodes from a starting point to an ending point.

* **Euclidean Distance:** The straight-line distance between two points. If you draw a line between two nodes, its length is the Euclidean distance. This is how we measure the "cost" of moving between nodes.

* **Target:** A specific `(X, Y)` coordinate that we want to reach. It might not be exactly on one of our `Nodes`.

* **"Closest Node to Target":** Since our `Target` might not be a node itself, Dijkstra's finds the shortest path to the *node that is geographically nearest* to the `Target` coordinate.

* **"Simple Path":** A path that does not visit the same node more than once. Both Dijkstra's and our DFS primarily find simple paths.

## What You'll See in the Output:

The program will print information to your console:

* **`Generated Target Position:`**: The `(X, Y)` coordinates of the random destination.

* **`Vector Contents (X elements):`**: A list of all the random nodes generated, showing their index `[0]`, `[1]`, etc., and their coordinates.

* **`Nearest node to Node 0...`**: Shows which node is closest to your starting node (Node 0).

* **`Starting Dijkstra's...`**: Progress messages from Dijkstra's as it finds the shortest path. It will tell you when it finds a "new closest node to target."

* **`Confirmed shortest path to the closest node to target:`**: The final result from Dijkstra's, confirming which node was closest to the target and its distance.

* **`Attempting to find up to X paths...`**: Messages from the limited DFS, indicating its search parameters.

* **`Found X paths within limits.`**: How many paths the limited DFS managed to find.

* **`=== Comparison of Paths ===`**: This is the main section for analysis.

  * **`--- All Paths Found (Limited DFS) ---`**: Each path found by DFS will be listed, along with its total length, and visualized in an ASCII "tree-like" format.

  * **`--- Shortest Path (Dijkstra's Result) ---`**: The single shortest path found by Dijkstra's, its total length, and its ASCII visualization. This is your optimal path.

  * **`--- Shortest Path Among Limited DFS Paths ---`**: From the sample of paths found by DFS, this section highlights the one that happened to be the shortest.

## Why Compare Dijkstra's and DFS?

This comparison is crucial for understanding:

* **Efficiency:** Dijkstra's is highly efficient for finding *the* shortest path. It doesn't waste time exploring paths that are clearly longer.

* **Completeness (for shortest path):** Dijkstra's guarantees finding the true shortest path (given non-negative edge weights).

* **Exploration:** DFS, when unconstrained, explores *all* possibilities, which is useful for tasks like finding all routes, but incredibly inefficient for just finding the shortest one. The "limited" DFS here gives you a taste of that exploration without crashing your computer.

By looking at the total lengths and the visualizations, you can see how Dijkstra's efficiently zeroes in on the best path, while DFS might explore many longer alternatives before stumbling upon a short one (if it finds it within its limits!).

## Code Structure (High-Level)

The program is organized into clear components:

* **`Node` Class:** Represents a single point in our 2D space and its connection to the next node in a linked list.

* **`LinkedList` Class:** A custom data structure to store multiple paths. Each "node" in this `LinkedList` actually holds an entire path (`std::vector<glm::vec2>`). It also contains the `printPathAsTree` method for visualization.

* **Utility Functions:** `randomFloat`, `randomVec2`, `GenerateRandomVectorList`, `printVector`, `calculateDistance`, `calculatePathLength`, `findNearestNode` help with setup and basic calculations.

* **`DijkstraNode` Struct:** A helper structure used internally by Dijkstra's algorithm.

* **`findShortestPathToClosest` Function:** Implements Dijkstra's algorithm.

* **`findAllPathsRecursive` & `findAllPathsLimited` Functions:** Implement the limited Depth-First Search.

* **`main` Function:** Orchestrates everything, setting up the nodes, running the algorithms, and printing the results.

## Contribution

Got ideas to make this "Fucktional" algorithm even more awesome? Feel free to fork this repository, open issues, or submit pull requests. All suggestions for improvements, optimizations, or even more creative ASCII art visualizations are welcome!

## License

[Choose a license, e.g., MIT, Apache 2.0, GPL, etc.]
