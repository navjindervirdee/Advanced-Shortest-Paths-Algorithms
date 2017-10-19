//Program to Implement A-Start Agorithm.

import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.lang.Math;

public class DistWithCoords {
    //class for a Vertex in the Graph.
    static class Vertex{
	int vertexNum;                 //id of the vertex.
	int x;                         //x co-ordinate of the vertex.
	int y;                         //y co-ordinate of the vertex.
	long distance;                 //distance of the vertex from the source. 
	long potential;                //euclidean distance of the vertex and target vertex.
	long distwithPotential;        //combining i.e suming distance and potential.
	int queuePos;                  //pos of the vertex in the PriorityQueue.
	boolean processed;             //check if the vertex is processed while traversing the graph. 
	ArrayList<Integer> adjList;    //list of adjacent vertices from this vertex.
	ArrayList<Integer> costList;   //list of costs or distances of adjacent vertices from this vertex.
	
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

    //Implementing PriorityQueue data structure by myself for this program. (using Min-Heap property.)
    static class PriorityQueue{
	//function to swap elements int the priorityQ.
	public void swap(Vertex [] graph, int [] priorityQ, int index1, int index2){
		int temp = priorityQ[index1];

		priorityQ[index1]=priorityQ[index2];
		graph[priorityQ[index2]].queuePos=index1;

		priorityQ[index2]=temp;
		graph[temp].queuePos=index2;
	}

	//function to swap the source vertex with the first element in the priorityQ.		
	public void makeQueue(Vertex [] graph,int [] forwpriorityQ, int source, int target){
		swap(graph, forwpriorityQ,0,source);
	}

	//function to extract the min element from the priorityQ. based on the distwithPotential attribute.
	public int extractMin(Vertex [] graph, int [] priorityQ, int extractNum){
		int vertex = priorityQ[0];
		int size = priorityQ.length-1-extractNum;
		swap(graph,priorityQ,0,size);
		siftDown(0,graph,priorityQ,size);
		return vertex;
	}

	//function to siftdown the element at the given index in the priorityQ.
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
	
	//function to change the priority of an element in the priorityQ. (priority can only decrease).
	public void changePriority(Vertex [] graph, int [] priorityQ, int index){
		if((index-1)/2 > -1 && graph[priorityQ[index]].distwithPotential < graph[priorityQ[(index-1)/2]].distwithPotential){
			swap(graph,priorityQ,index,(index-1)/2);
			changePriority(graph,priorityQ,(index-1)/2);
		}
	}
    }
	

    //function to calculate the euclidean distance between two vertices.
    private static long calcPotential(Vertex [] graph, int vertex1, int vertex2){
	long potential = (long)Math.sqrt(Math.pow((graph[vertex1].x - graph[vertex2].x),2) + Math.pow((graph[vertex1].y - graph[vertex2].y),2));
	return potential;
    }

    //function to initialize the graph.
    public static void initialize(Vertex [] graph, int [] forwpriorityQ, int source, int target){
	for(int i=0;i<graph.length;i++){
		graph[i].processed = false;
		graph[i].distance = Long.MAX_VALUE;
		graph[i].distwithPotential = Long.MAX_VALUE;
		graph[i].potential = calcPotential(graph, i, target);
		forwpriorityQ[i]=i;
	    	graph[i].queuePos=i;
		
	}
	graph[source].distance = 0;
	graph[source].distwithPotential = 0;
	
	
    }


    //function to relax the edges i.e process every adjacent edge of the given vertex.
    private static void relaxEdges(Vertex [] graph, int [] priorityQ, int vertex, PriorityQueue queue){
	ArrayList<Integer> vertexList = graph[vertex].adjList;
	ArrayList<Integer> costList = graph[vertex].costList;
	graph[vertex].processed = true;
	
	for(int i=0;i<vertexList.size();i++){
		int temp =vertexList.get(i);
		int cost = costList.get(i);
		
		if(graph[temp].distance > graph[vertex].distance + cost){
			graph[temp].distance = graph[vertex].distance + cost;
			graph[temp].distwithPotential = graph[temp].distance + graph[temp].potential;
			queue.changePriority(graph,priorityQ,graph[temp].queuePos);
		}
	}
    }
 

    //function to compute the distance between soure and the target.
    public static long computeDist(Vertex [] graph, int source, int target){
	//create priorityQ.
	int [] forwpriorityQ = new int[graph.length];

	//initialize the graph.
	initialize(graph,forwpriorityQ,source,target);	

	PriorityQueue queue = new PriorityQueue();
	queue.makeQueue(graph, forwpriorityQ,source,target);

	for(int i=0;i<graph.length;i++){
		//extact the element with the min
		int vertex1 = queue.extractMin(graph,forwpriorityQ,i);
		
		if(graph[vertex1].distance == (Long.MAX_VALUE)){
			return -1;
		}
		
		//if target vertex found return the distance.
		if(vertex1==target){
			return graph[vertex1].distance;
		}
		
		//else relax the edges of the extracted vertex.
		relaxEdges(graph,forwpriorityQ,vertex1,queue);
		
	}
	
	//if no path between source and target vertex.
	return -1;
    }

    
    //main function to run the program.
    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
        int n = in.nextInt();   //number of vertices in the graph.
        int m = in.nextInt();   //number of edges in the graph.

	//create the graph.
	Vertex [] graph = new Vertex[n];  

	//get the co-ordinates of every vertex.	
	for (int i = 0; i < n; i++) { 
            int x, y;
            x = in.nextInt();
            y = in.nextInt();
	    graph[i] = new Vertex(i,x,y);
	}

	//get the edges in the graph.
        for (int i = 0; i < m; i++) {
            int x, y, c;
            x = in.nextInt();
            y = in.nextInt();
            c = in.nextInt();

	    graph[x-1].adjList.add(y-1);
	    graph[x-1].costList.add(c);

	}

	//number of queries.
        int q = in.nextInt();

        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;
            t = in.nextInt()-1;
            System.out.println(computeDist(graph,s,t));
        }
    }
}