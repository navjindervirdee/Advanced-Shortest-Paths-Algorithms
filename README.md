# Advanced-Shortest-Paths-Algorithms
Java Code for Contraction Hierarchies Algorithm, A-Star Algorithm and Bidirectional Dijkstra Algorithm. Tested and Verified Code.

* **Bi-Directional Dijsktra Algorithm**: Bidirectional search is a graph search algorithm that finds a shortest path from an initial vertex to a goal vertex in a directed graph. It runs two simultaneous searches: one forward from the initial state, and one backward from the goal, stopping when the two meet in the middle. The reason for this approach is that in many cases it is faster.

  * **Algorithm**:
    * Read the vertices and edges and create a graph using adjacency list.
    * Create a reverse graph from the input graph.
    * Start Dijkstra from source vertex in the forward graph and from target vertex in the reverse graph.
    * Alternate between Dijkstra steps in forward and reverse graph.
    * Stop! whwn some vertex v is processed both in forward and reverse graph.
    * For all the vertices processed in the forward and reverse graph, calculate distance from source to that vertex plus vertex to target
    * Print the min distance from the above step.

* **A-Star Algorithm**: In computer science, A* (pronounced as "A star") is a computer algorithm that is widely used in pathfinding and graph traversal, the process of plotting an efficiently directed path between multiple points, called nodes. It enjoys widespread use due to its performance and accuracy. However, in practical travel-routing systems, it is generally outperformed by algorithms which can pre-process the graph to attain better performance, although other work has found A* to be superior to other approaches.

  What A* Search Algorithm does is that at each step it picks the node according to a value-‘f’ which is a parameter equal to the sum of     two other parameters – ‘g’ and ‘h’. At each step it picks the node/cell having the lowest ‘f’, and process that node/cell.

  We define ‘g’ and ‘h’ as simply as possible below

  g = the minimum distance to move from the starting point to a given node in the graph.
  h = It is nothing but the eulidean distance between the current node/vertex and the target vertex. Other heuristics can also be used.
  
* **Contraction Hierarchies Algorithm**:  In applied mathematics, the method of contraction hierarchies is a technique to speed up shortest-path routing by first creating precomputed "contracted" versions of the connection graph. It can be regarded as a special case of "highway-node routing".

  Contraction hierarchies can be used to generate shortest-path routes much more efficiently than Dijkstra's algorithm or previous           highway-node routing approaches, and is used in many advanced routing techniques.It is publicly available in open source software to       calculate routes from one place to another.
  
    * **Algorithm**:
      * Read the input vertices and edges and create the graph.
      * Pre-process the graph.
      * Use modified Bidirectional Dijkstra to find shortest distance between source and the target.
      
      **Pre-Processing**:
      
        **Order By Importance**: Introduce a measure of importance and contract the least importance vertices. Importance can also               change after we contract the vertex i.e importance of adjacent vertices may change due to contraction of this vertex.
      
      **Steps**:
        * Keep all the vertices in a priority queue by decreasing importance.
        * On each iteration, extract the least important vertex.
        * Recompute its importance because it could have changed because of the contraction of other nodes.
        * If its still minimum contract the node.
        * Otherwise, put it back into the priroity queue.
        * Store these contracted vertices in increasing order i.e vertex contracted first comes first and contracted last comes at end.
        
      **Importance Criteria**:
        * Edge Difference : number of added shortcuts - number of incoming edges - number of outgoing egdes.
        * Contracted Neighbors : Want to spread the contracted vertices across the graph. Contract the node with small no. of contracted           neighbors.
        * Shortcut Cover: Number of neighbors w of v such that we have to shortcut to or from after contracting v.
        
      **Modified Bidirectional Dijkstra**:
        * Bidirectional Dijkstra will use only upward edges i.e edges going from vertices contracted first to vertices contracted later.
        * Don't stop when some node was processed in both forward and backward searches.
        * Stop when the processed node's distance is already farther than the target vertex. i.e keep on processing other nodes until             there are no more nodes to process both in forward search as well as backward search.
