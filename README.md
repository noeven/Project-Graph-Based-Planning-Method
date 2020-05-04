# Project-Graph-Based-Planning-Method
The goal of this project was design two different graph based path planning algorithms in order to evaluate their performance.
The planning system are implemented by Dijkstra and A* algorithm, and it works on 2D grid like environment. For both scripts the input map will be specified as 2D logical array where the false or zero entries correspond to free cells and the true or non-zero entries correspond to obstacle cells. The goal of these planning routines is to construct a route between the specified start and destination cells.

The skeleton code provided contains the following 2D arrays:
- map: an array of integer values which indicates the current status of each cell in the grid. Different integer values are used to indicate the different possible states listed below:
Whether the node is part of an obstacle
  - If the node is in freespace,
  - Whether or not is has been visited,
  - Is currently on the list of nodes being considered.

- distanceFromStart: this array encodes the current estimated for the distance between each node and the start node.
- parent: For every cell in the map this array records the index of its parent with respect to the algorithm, that is the next node along the shortest path to the start node.
