//Program to implement Bidirectional A-Star Algorithm.

import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.lang.Math;

public class DistWithCoords {

    //class Vertex of the Graph.    
    static class Vertex{
	int vertexNum;                  //id of the vertex.
	int x;				//x co-oridnate of the vertex.
	int y;				//y co-ordinate of the vertex.
	long distance;			//distance of this vertex from the source vertex.
	long potential;			//euclidean distance of this vertex and target vertex.
	long distwithPotential;		//summing potential and distance
	int queuePos;			//pos of this vertex in the PriorityQueue.
	boolean processed;		//check if processed while traversing the graph.
	ArrayList<Integer> adjList;	//list of adjacent vertices from this vertex.
	ArrayList<Integer> costList;	//list of costs or distances of adjacent vertices from this vertex.
	
	public Vertex(){
	}

	public Vertex(int vertexNum, int x, int y){
		this.vertexNum=vertexNum;
		this.x=x;
		this.y=y;
		this.adjList=new ArrayList<Integer>();
		this.costList=new ArrayList<Integer>();
	}

    }


    //Implemented PriorityQueue Data Structure by myself.(using Min-Heap Property)
    static class PriorityQueue{
	//function to swap values in the priorityQ.
	public void swap(Vertex [] graph, int [] priorityQ, int index1, int index2){
		int temp = priorityQ[index1];

		priorityQ[index1]=priorityQ[index2];
		graph[priorityQ[index2]].queuePos=index1;

		priorityQ[index2]=temp;
		graph[temp].queuePos=index2;
	}
		
	//function to swap source vertex with the first vertex int he priorityQ.
	public void makeQueue(Vertex [] graph, Vertex [] reverseGraph, int [] forwpriorityQ, int [] revpriorityQ, int source, int target){
		swap(graph, forwpriorityQ,0,source);
		swap(reverseGraph, revpriorityQ,0,target);
	}

	//function to extract the vertex with min distwithpotential value from the PriorityQueue.
	public int extractMin(Vertex [] graph, int [] priorityQ, int extractNum){
		int vertex = priorityQ[0];
		int size = priorityQ.length-1-extractNum;
		swap(graph,priorityQ,0,size);
		siftDown(0,graph,priorityQ,size);
		return vertex;
	}


	//function to siftdown the vertex in the priotityQ.
	public void siftDown(int index, Vertex [] graph, int [] priorityQ, int size){
		int min = index;
		if(2*index+1<size && graph[priorityQ[index]].distwithPotential > graph[priorityQ[2*index+1]].distwithPotential){
			min = 2*index+1;
		}
		if(2*index+2<size && graph[priorityQ[min]].distwithPotential > graph[priorityQ[2*index+2]].distwithPotential){
			min = 2*index+2;
		}
		if(min!=index){
			swap(graph,priorityQ,min,index);
			siftDown(min,graph,priorityQ,size);
		}
	}
	
	//function to change the prirority of a vertex. (can only decrease the priority).
	public void changePriority(Vertex [] graph, int [] priorityQ, int index){
		if((index-1)/2 > -1 && graph[priorityQ[index]].distwithPotential < graph[priorityQ[(index-1)/2]].distwithPotential){
			swap(graph,priorityQ,index,(index-1)/2);
			changePriority(graph,priorityQ,(index-1)/2);
		}
	}
    }


    //function to calculate the potential of the vertex i.e euclidean distance between two vertices.		
    private static long calcPotential(Vertex [] graph, int vertex1, int vertex2){
	long potential = (long)Math.sqrt(Math.pow((graph[vertex1].x - graph[vertex2].x),2) + Math.pow((graph[vertex1].y - graph[vertex2].y),2));
	return potential;
    }

	
    //function to initialize the graph.
    public static void initialize(Vertex [] graph, Vertex [] reverseGraph, int [] forwpriorityQ, int [] revpriorityQ, int source, int target){
	for(int i=0;i<graph.length;i++){
		graph[i].processed = false;
		graph[i].distance = Long.MAX_VALUE;
		graph[i].distwithPotential = Long.MAX_VALUE;
		graph[i].potential = (calcPotential(graph, i, target) - calcPotential(graph,i,source))/2;
		forwpriorityQ[i]=i;
		graph[i].queuePos=i;
		
		reverseGraph[i].processed = false;
		reverseGraph[i].distance = (Long.MAX_VALUE);
		reverseGraph[i].distwithPotential = (Long.MAX_VALUE);
		reverseGraph[i].potential = (calcPotential(reverseGraph,i,source)-calcPotential(reverseGraph,i,target))/2;
		revpriorityQ[i]=i;
		reverseGraph[i].queuePos=i;
	}
	graph[source].distance = 0;
	graph[source].distwithPotential = 0;
	reverseGraph[target].distance = 0;
	reverseGraph[target].distwithPotential=0;
    }


    //function to relax the edges of the given vertex i.e process the adjacent edges.
    private static void relaxEdges(Vertex [] graph, int [] priorityQ, int vertex, PriorityQueue queue){
	ArrayList<Integer> vertexList = graph[vertex].adjList;
	ArrayList<Integer> costList = graph[vertex].costList;
	graph[vertex].processed = true;
	
	for(int i=0;i<vertexList.size();i++){
		int temp =vertexList.get(i);
		int cost = costList.get(i);
		
		if(graph[temp].distance > graph[vertex].distance + cost){
			graph[temp].distance = graph[vertex].distance + cost;
			graph[temp].distwithPotential = graph[temp].distance + graph[temp].potential ;
			queue.changePriority(graph,priorityQ,graph[temp].queuePos);
		}
	}
    }

    
    //function to find the correct distance of the vertex.
    public static void correctDistance(Vertex [] graph, Vertex [] reverseGraph, int vertex, long correctDist){
	if(graph[vertex].distance == Long.MAX_VALUE || reverseGraph[vertex].distance == Long.MAX_VALUE){
		return;
	}
	if(correctDist>graph[vertex].distance + reverseGraph[vertex].distance){
		correctDist = graph[vertex].distance + reverseGraph[vertex].distance;
	}
    } 

    //function to compute the distance between the source vertex and the target vertex.
    public static long computeDist(Vertex [] graph, Vertex [] reverseGraph, int source, int target){
	//create the PriorityQueue's
	int [] forwpriorityQ = new int[graph.length];   //for forward propagation
	int [] revpriorityQ = new int[graph.length];    //for reverse graph i.e backward propagation.
	 
	//initialize the graph.
	initialize(graph,reverseGraph,forwpriorityQ,revpriorityQ,source,target);	

	PriorityQueue queue = new PriorityQueue();
	queue.makeQueue(graph, reverseGraph, forwpriorityQ, revpriorityQ, source,target);

	ArrayList<Integer> forwprocessedVertices = new ArrayList<Integer>();   //list to store the processed vertices in the forward propagation.
	ArrayList<Integer> revprocessedVertices = new ArrayList<Integer>();    //list to store the processed vertices in the reverse graph. i.e backward propagation.
	long correctDist = Long.MAX_VALUE;
	
	for(int i=0;i<graph.length;i++){
		int vertex1 = queue.extractMin(graph,forwpriorityQ,i);          //extract the min vertex from the forward graph.
		int vertex2 = queue.extractMin(reverseGraph, revpriorityQ, i);  //extract the min vertex from the reverse graph.
		
		if(graph[vertex1].distance == (Long.MAX_VALUE) || reverseGraph[vertex2].distance == (Long.MAX_VALUE)){
			return -1;
		}
		
		//relax the edges. 
		relaxEdges(graph,forwpriorityQ,vertex1,queue);
		
		//store the vertex in the forward list.
		forwprocessedVertices.add(vertex1);

		//find the correct distance.
		correctDistance(graph,reverseGraph,vertex1,correctDist);

		//if also processed in the reverse graph then compute shortest distance.
		if(graph[vertex1].processed && reverseGraph[vertex1].processed){
			return shortestPath(graph,reverseGraph,forwprocessedVertices, revprocessedVertices,vertex1, correctDist);
		}

		//relax the edges of the min vertex in the reverse graph.
		relaxEdges(reverseGraph,revpriorityQ,vertex2,queue);
		
		//add into the reverse processed list.
		revprocessedVertices.add(vertex2);

		//compute the correct the distance.
		correctDistance(graph,reverseGraph,vertex2,correctDist);

		//if processed in the forward graph, compute the shortest distance.
		if(reverseGraph[vertex2].processed && graph[vertex2].processed){
			return shortestPath(graph,reverseGraph,forwprocessedVertices, revprocessedVertices,vertex2,correctDist);
		}
	}

	//if no path between sorce vertex and target vertex.
	return -1;
    }

   
    //function to compute the shortest path of all the processed vertives in both forward and reverse propagation.
    public static long shortestPath(Vertex [] graph, Vertex [] reverseGraph, ArrayList<Integer> forwprocessedVertices, ArrayList<Integer> revprocessedVertices, int vertex,long correctDist){
	long distance = Long.MAX_VALUE;

	//process the list of forward processed vertices.
	for(int i=0;i<forwprocessedVertices.size();i++){
		int temp = forwprocessedVertices.get(i);
		if(reverseGraph[temp].distance != Long.MAX_VALUE && distance > graph[temp].distance + reverseGraph[temp].distance){
			distance = graph[temp].distance + reverseGraph[temp].distance;
		}
	}

	//process the list of reverse processed vertices.
	for(int i=0;i<revprocessedVertices.size();i++){
		int temp = revprocessedVertices.get(i);
		if(graph[temp].distance != Long.MAX_VALUE && distance > graph[temp].distance + reverseGraph[temp].distance){
			distance = graph[temp].distance + reverseGraph[temp].distance;
		}
	}
		
	return distance;
    }
		

    //main function to run the program.	
    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
	System.out.println("Enter the number of vertices and edges.");
        int n = in.nextInt();   //number of vertices.
        int m = in.nextInt();   //number of edges.
	
	//create forward and reverse graph.
	Vertex [] graph = new Vertex[n];
	Vertex [] reverseGraph = new Vertex[n];
	
	//get the co-ordinates of the vertices.
	System.out.println("Enter the Coordinates.");
	for (int i = 0; i < n; i++) { 
            int x, y;
            x = in.nextInt();   //x co-or
            y = in.nextInt();   //y co-or
	
	    graph[i] = new Vertex(i,x,y);
	    reverseGraph[i] = new Vertex(i,x,y);
        }

	
	//get the edges in the graph.
	System.out.println("Enter the edges with weights (V1 V2 W).");
        for (int i = 0; i < m; i++) {
            int x, y, c;
            x = in.nextInt();
            y = in.nextInt();
            c = in.nextInt();

	    graph[x-1].adjList.add(y-1);
	    graph[x-1].costList.add(c);

	    reverseGraph[y-1].adjList.add(x-1);
	    reverseGraph[y-1].costList.add(c);
	}

	System.out.println("Enter the number of queries.");
        int q = in.nextInt(); //number of queries

	System.out.println("Enter the queries (S T)");
        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;   //source vertex.
            t = in.nextInt()-1;   //target vertex.
            System.out.println(computeDist(graph,reverseGraph,s,t));
        }
    }
}