# Advanced-Shortest-Paths-Algorithms
Java Code for Contraction Hierarchies Algorithm, A-Star Algorithm and Bidirectional Dijkstra Algorithm. Tested and Verified Code.

* **Bi-Directional Dijsktra Algorithm**: Bidirectional search is a graph search algorithm that finds a shortest path from an initial vertex to a goal vertex in a directed graph. It runs two simultaneous searches: one forward from the initial state, and one backward from the goal, stopping when the two meet in the middle. The reason for this approach is that in many cases it is faster.

* **A-Star Algorithm**: In computer science, A* (pronounced as "A star") is a computer algorithm that is widely used in pathfinding and graph traversal, the process of plotting an efficiently directed path between multiple points, called nodes. It enjoys widespread use due to its performance and accuracy. However, in practical travel-routing systems, it is generally outperformed by algorithms which can pre-process the graph to attain better performance, although other work has found A* to be superior to other approaches.

  What A* Search Algorithm does is that at each step it picks the node according to a value-‘f’ which is a parameter equal to the sum of     two other parameters – ‘g’ and ‘h’. At each step it picks the node/cell having the lowest ‘f’, and process that node/cell.

  We define ‘g’ and ‘h’ as simply as possible below

  g = the minimum distance to move from the starting point to a given node in the graph.
  h = It is nothing but the eulidean distance between the current node/vertex and the target vertex. Other heuristics can also be used.
