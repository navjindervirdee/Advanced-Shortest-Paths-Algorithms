# Advanced-Shortest-Paths-Algorithms
Java Code for Contraction Hierarchies Algorithm, A-Star Algorithm and Bidirectional Dijkstra Algorithm. Tested and Verified Code.


This project was part of Algorithms on Graphs course on Coursera. The task was to implement algorithms for finding the shortest paths between any vertices in the graph. Implemented the following algorithms in java language:


•	Contraction Hierarchies Algorithm
•	A-Star Algorithm
•	Bidirectional  Dijkstra Algorithm


Contraction Hierarchies Algorithm: In this task I was first given a graph of a real road network, and I had to pre-process it under the pre-processing time limit. Then I got a set of queries for computing distance, and I needed to answer all of them under the separate time limit for queries. 

This algorithm was tested on a graph with 110,000 vertices and 250,000 edges on 10,000 queries with total pre-processing time of 30sec and total running time for all queries equal to 6sec.



A-Star Algorithm: In this task I was given a description of a real-world road network with not just edges and their lengths, but also with the coordinates of the nodes. My task was still to find the distance between some pairs of nodes, but I needed to use the additional information about coordinates to speed up my search.

This algorithm was tested on a graph with 110,000 vertices and 250,000 edges on 10,000 queries with total running time of all queries equal to 90sec. 



Bidirectional Dijkstra Algorithm: Social networks are live on the connections between people, so friend suggestions is one of the most important features of Facebook. One of the most important inputs of the algorithm for friend suggestion is most probably the current distance between you and the suggested person in the graph of friend’s connections. My task was to implement efficient computation of this distance. The grader had tested the algorithm against different real-world networks, such as a part of internet, a network of scientific citations or co-authorship, a social network of jazz musicians or even a social network of dolphins. I needed to compute the distance between two nodes in such network.

This algorithm was tested on a graph with 1,000,000 vertices and 6,000,000 edges on 1000 queries with total running time of all queries equal to 90 sec. 
