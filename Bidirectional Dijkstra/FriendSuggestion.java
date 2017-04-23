import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Comparator;

public class FriendSuggestion {
	
    static class Vertex{
	int vertexNum;
	ArrayList<Integer> adjList; //adjacent vertices of the vertex.
	ArrayList<Integer> costList; //cost of adjacent edges of the vertex.

	int queuePos; //position of the vertex in the priorityQueue.
	long dist; //distance is stored.
	boolean processed; //is this vertex processed.

	public Vertex(){
	}

	public Vertex(int vertexNum){
		this.vertexNum=vertexNum;
		this.adjList = new ArrayList<Integer>();
		this.costList = new ArrayList<Integer>();
	}

	//initializes the instance variables.
	public void createGraph(Vertex [] graph, Vertex [] reverseGraph, int [] forwPriorityQ, int [] revPriorityQ){
		for(int i=0;i<graph.length;i++){
			graph[i].queuePos=i;
			graph[i].processed=false;
			graph[i].dist = Integer.MAX_VALUE;
		
			reverseGraph[i].queuePos =i;
			reverseGraph[i].processed=false;
			reverseGraph[i].dist = Integer.MAX_VALUE;

			forwPriorityQ[i]=i;
			revPriorityQ[i]=i;
		}
	}

    }

  
    //implemented the priorityQueue by myself.
    static class PriorityQueue{
	
	//swap function maintaining the queuePos variable.
	public void swap(Vertex [] graph, int [] priorityQ, int index1, int index2){
		int temp = priorityQ[index1];

		priorityQ[index1]=priorityQ[index2];
		graph[priorityQ[index2]].queuePos=index1;

		priorityQ[index2]=temp;
		graph[temp].queuePos=index2;
	}
		
	//makes the queue just put the source vertex at position zero ,target vertex has no role.
	public void makeQueue(Vertex [] graph,int [] forwpriorityQ, int source, int target){
		swap(graph, forwpriorityQ,0,source);
	}


	//extract the min elements i.e the first element.
	public int extractMin(Vertex [] graph, int [] priorityQ, int extractNum){
		int vertex = priorityQ[0];
		int size = priorityQ.length-1-extractNum;
		swap(graph,priorityQ,0,size);
		siftDown(0,graph,priorityQ,size);
		return vertex;
	}

	
	//sift down the larger elements downwards.
	public void siftDown(int index, Vertex [] graph, int [] priorityQ, int size){
		int min = index;
		if(2*index+1<size && graph[priorityQ[index]].dist > graph[priorityQ[2*index+1]].dist){
			min = 2*index+1;
		}
		if(2*index+2<size && graph[priorityQ[min]].dist > graph[priorityQ[2*index+2]].dist){
			min = 2*index+2;
		}
		if(min!=index){
			swap(graph,priorityQ,min,index);
			siftDown(min,graph,priorityQ,size);
		}
	}
	
	//changes priority of the vertex.
	public void changePriority(Vertex [] graph, int [] priorityQ, int index){
		if((index-1)/2 > -1 && graph[priorityQ[index]].dist < graph[priorityQ[(index-1)/2]].dist){
			swap(graph,priorityQ,index,(index-1)/2);
			changePriority(graph,priorityQ,(index-1)/2);
		}
	}
    }


    //function to relax the edges of the vertex.   
    private static void relaxEdges(Vertex [] graph, int vertex, int [] priorityQ, PriorityQueue queue,int queryId){
	ArrayList<Integer> vertexList = graph[vertex].adjList;
	ArrayList<Integer> costList = graph[vertex].costList;
	graph[vertex].processed = true;
	
	for(int i=0;i<vertexList.size();i++){
		int temp = vertexList.get(i);
		int cost = costList.get(i);
		
		if(graph[temp].dist>graph[vertex].dist + cost){
			graph[temp].dist = graph[vertex].dist + cost;	
			queue.changePriority(graph,priorityQ,graph[temp].queuePos);
		}
	}
    }


    //bidirectional dijkstra implemented.
    public static long computeDistance(Vertex [] graph, Vertex [] reverseGraph, int s, int t,int queryId){
	PriorityQueue queue = new PriorityQueue(); //created priotityQueue object.

	int [] forwPriorityQ = new int[graph.length]; //priorityQ for forward search.
	int [] revPriorityQ = new int[graph.length]; //priorityQ for backward search

	Vertex vertex = new Vertex();
	vertex.createGraph(graph,reverseGraph,forwPriorityQ,revPriorityQ); //reinitializes the graph for each query.

	graph[s].dist=0; //initialized source distance zero.
	reverseGraph[t].dist=0; //initialized target distance to zero.
	
	queue.makeQueue(graph,forwPriorityQ,s,t); //makes forwardQ.
	queue.makeQueue(reverseGraph,revPriorityQ,t,s); // makes revQ.

	ArrayList<Integer> forgraphprocessedVertices = new ArrayList<Integer>(); //to store vertices processed in the forward search.
	ArrayList<Integer> revgraphprocessedVertices = new ArrayList<Integer>(); //to store vertices processed in the backward search.
	
	for(int i=0;i<graph.length;i++){
		int vertex1 = queue.extractMin(graph,forwPriorityQ,i); //extracted the vertex with min distance.
		if(graph[vertex1].dist==Integer.MAX_VALUE){
			continue;
		}
		relaxEdges(graph,vertex1,forwPriorityQ,queue,queryId); //relaxed the edges.
		forgraphprocessedVertices.add(vertex1); //added in the forward lsit.

		//check wether we have found vertex processed both in forward and backward then return the shortest path.
		if(reverseGraph[vertex1].processed){
			return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
		}
		

		int revVertex = queue.extractMin(reverseGraph,revPriorityQ,i); //extracted min distance vertex in backward search.
		if(reverseGraph[revVertex].dist==Integer.MAX_VALUE){
			continue;
		}
		relaxEdges(reverseGraph,revVertex,revPriorityQ,queue,queryId); //relaxed edges in the backward search.
		revgraphprocessedVertices.add(revVertex); //added the vertex to backward search processed list.
		
		//check whether the vertex is processed in forward search as well then return shortest distance.
		if(graph[revVertex].processed){
			return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
		}
	}
	
	return -1;
    }


    //function to calculate shortest distance.
    private static long shortestPath(Vertex [] graph, Vertex [] reverseGraph, ArrayList<Integer> forgraphprocessedVertices, ArrayList<Integer> revgraphprocessedVertices,int queryId){
	long distance = Integer.MAX_VALUE;
	
	for(int i=0;i<forgraphprocessedVertices.size();i++){
		int vertex = forgraphprocessedVertices.get(i);
		if(reverseGraph[vertex].dist + graph[vertex].dist>=Integer.MAX_VALUE){
			continue;
		}
		long tempdist = graph[vertex].dist + reverseGraph[vertex].dist;	
		if(distance>tempdist){
			distance=tempdist;
		}
	}
	
	for(int i=0;i<revgraphprocessedVertices.size();i++){
		int vertex = revgraphprocessedVertices.get(i);
		if(reverseGraph[vertex].dist + graph[vertex].dist>=Integer.MAX_VALUE){
			continue;
		}
		long tempdist = reverseGraph[vertex].dist + graph[vertex].dist;
		if(distance>tempdist){
			distance=tempdist;
		}
	
	}
	return distance;
    }
    


    //main function to drive the program.
    public static void main(String args[]) {
	Scanner in = new Scanner(System.in);
        int n = in.nextInt(); //number of vertices.
	int m = in.nextInt(); //number of edges.

	Vertex vertex = new Vertex(); 
	Vertex [] graph = new Vertex[n]; //graph for forward search.
	Vertex [] reverseGraph = new Vertex[n]; //graph for backward search.

	for(int i=0;i<n;i++){
		graph[i]=new Vertex(i);
		reverseGraph[i]=new Vertex(i);
	}
	
	for(int i=0;i<m;i++){
		int u, v;
		int w;
		u= in.nextInt();
		v=in.nextInt(); 
		w=in.nextInt();

		graph[u-1].adjList.add(v-1);
		graph[u-1].costList.add(w);

		reverseGraph[v-1].adjList.add(u-1);
		reverseGraph[v-1].costList.add(w);
	}

	int q = in.nextInt(); //number of queries.

        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;
            t = in.nextInt()-1;
		
	    System.out.println(computeDistance(graph,reverseGraph,s,t,i));
	}
	
    }
}
