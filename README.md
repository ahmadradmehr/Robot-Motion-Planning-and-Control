# Robot Motion Planning and Control
This repository is related to the a course offered by Northwestern University at Coursera.

## Projects
### A* Algorithm
In this programming assignment, A* search is implemented. Given a graph, the start node, and the goal node, the program searches the graph for a minimum-cost path from the start to the goal. The program either returns a sequence of nodes for a minimum-cost path or indicates that no solution exists.  

The program successfully reads the nodes, edges, and obstacles data and coordinates from seperated .CSV files and finally writes the found minimum-cost-path to another .CSV file.
CoppeliaSim is used to visualize the graph and the solution path found by the program.  
### RRT Algorithm
In this programming assignment, the RRT sampling-based planner is implemented to find a path for a point robot through the cluttered planar environment in CoppeliaSim.  
The program chooses its own random samples in the (x, y) C-space, which is the square [-0.5, 0.5] x [-0.5, 0.5]. A straight-line planner is employed as the local planner. The distance between two configurations is simply the Euclidean distance.

The start configuration is at (x, y) = (-0.5, -0.5), the bottom left corner of the square environment, and the goal configuration is at (x, y) = (0.5, 0.5), the top right corner of the square environment.

The program is responsible for collision checking. Given a straight line segment from (x1, y1) to (x2, y2), a collision checker function is written to see if the line segment intersects a circle (all the obstacles are circles). A less desirable, but still acceptable, solution could test sample points finely on the line segment between (x1, y1) and (x2, y2).

The program should take as input the obstacles.csv containing the obstacles radii and positions. The output of the program is three files: nodes.csv, edges.csv, and path.csv.

### PRM Algorithm
In this programming assignment, the PRM sampling-based planner is implemented to find a path for a point robot through the cluttered planar environment in CoppeliaSim.  

First a graph is built representing the C-space before searching it. Then, the A* search algorithm is used to find the shortest path.

Phase 1, sampling: A uniform random distribution is sampled over the square [-0.5, 0.5] x [-0.5, 0.5].

Phase 2, creating edges: The number of neighbors k is determined as a varibale to try to connect the k neigbors to each node.

Phase 3, searching the graph: A* search is used.

The program chooses its own random samples in the (x, y) C-space, which is the square [-0.5, 0.5] x [-0.5, 0.5]. A straight-line planner is employed as the local planner. The distance between two configurations is simply the Euclidean distance.

The start configuration is at (x, y) = (-0.5, -0.5), the bottom left corner of the square environment, and the goal configuration is at (x, y) = (0.5, 0.5), the top right corner of the square environment.

The program is responsible for collision checking. Given a straight line segment from (x1, y1) to (x2, y2), a collision checker function is written to see if the line segment intersects a circle (all the obstacles are circles). A less desirable, but still acceptable, solution could test sample points finely on the line segment between (x1, y1) and (x2, y2).

The program should take as input the obstacles.csv containing the obstacles radii and positions. The output of the program is three files: nodes.csv, edges.csv, and path.csv.
