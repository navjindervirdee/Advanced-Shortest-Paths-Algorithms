import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.lang.Math;

public class DistWithCoords {
    
    static class Vertex{
	int vertexNum;
	int x;
	int y;
	long distance;
	long potential;
	long distwithPotential;
	int queuePos;
	boolean processed;
	ArrayList<Integer> adjList;
	ArrayList<Integer> costList;
	
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

    static class PriorityQueue{
	public void swap(Vertex [] graph, int [] priorityQ, int index1, int index2){
		int temp = priorityQ[index1];

		priorityQ[index1]=priorityQ[index2];
		graph[priorityQ[index2]].queuePos=index1;

		priorityQ[index2]=temp;
		graph[temp].queuePos=index2;
	}
		
	public void makeQueue(Vertex [] graph, Vertex [] reverseGraph, int [] forwpriorityQ, int [] revpriorityQ, int source, int target){
		swap(graph, forwpriorityQ,0,source);
		swap(reverseGraph, revpriorityQ,0,target);
	}

	public int extractMin(Vertex [] graph, int [] priorityQ, int extractNum){
		int vertex = priorityQ[0];
		int size = priorityQ.length-1-extractNum;
		swap(graph,priorityQ,0,size);
		siftDown(0,graph,priorityQ,size);
		return vertex;
	}

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
	
	public void changePriority(Vertex [] graph, int [] priorityQ, int index){
		if((index-1)/2 > -1 && graph[priorityQ[index]].distwithPotential < graph[priorityQ[(index-1)/2]].distwithPotential){
			swap(graph,priorityQ,index,(index-1)/2);
			changePriority(graph,priorityQ,(index-1)/2);
		}
	}
    }
		
    private static long calcPotential(Vertex [] graph, int vertex1, int vertex2){
	long potential = (long)Math.sqrt(Math.pow((graph[vertex1].x - graph[vertex2].x),2) + Math.pow((graph[vertex1].y - graph[vertex2].y),2));
	return potential;
    }

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
    
    public static void correctDistance(Vertex [] graph, Vertex [] reverseGraph, int vertex, long correctDist){
	if(graph[vertex].distance == Long.MAX_VALUE || reverseGraph[vertex].distance == Long.MAX_VALUE){
		return;
	}
	if(correctDist>graph[vertex].distance + reverseGraph[vertex].distance){
		correctDist = graph[vertex].distance + reverseGraph[vertex].distance;
		//System.out.println(correctDist);
	}
    } 

    public static long computeDist(Vertex [] graph, Vertex [] reverseGraph, int source, int target){
	int [] forwpriorityQ = new int[graph.length];
	int [] revpriorityQ = new int[graph.length];
	 
	initialize(graph,reverseGraph,forwpriorityQ,revpriorityQ,source,target);	

	PriorityQueue queue = new PriorityQueue();
	queue.makeQueue(graph, reverseGraph, forwpriorityQ, revpriorityQ, source,target);

	ArrayList<Integer> forwprocessedVertices = new ArrayList<Integer>();
	ArrayList<Integer> revprocessedVertices = new ArrayList<Integer>();
	long correctDist = Long.MAX_VALUE;
	
	for(int i=0;i<graph.length;i++){
		int vertex1 = queue.extractMin(graph,forwpriorityQ,i);
		int vertex2 = queue.extractMin(reverseGraph, revpriorityQ, i);
		
		if(graph[vertex1].distance == (Long.MAX_VALUE) || reverseGraph[vertex2].distance == (Long.MAX_VALUE)){
			return -1;
		}
		
		relaxEdges(graph,forwpriorityQ,vertex1,queue);
		forwprocessedVertices.add(vertex1);
		correctDistance(graph,reverseGraph,vertex1,correctDist);
		if(graph[vertex1].processed && reverseGraph[vertex1].processed){
			return shortestPath(graph,reverseGraph,forwprocessedVertices, revprocessedVertices,vertex1, correctDist);
		}

		relaxEdges(reverseGraph,revpriorityQ,vertex2,queue);
		revprocessedVertices.add(vertex2);
		correctDistance(graph,reverseGraph,vertex2,correctDist);
		if(reverseGraph[vertex2].processed && graph[vertex2].processed){
			return shortestPath(graph,reverseGraph,forwprocessedVertices, revprocessedVertices,vertex2,correctDist);
		}
	}
	return -1;
    }

    public static long shortestPath(Vertex [] graph, Vertex [] reverseGraph, ArrayList<Integer> forwprocessedVertices, ArrayList<Integer> revprocessedVertices, int vertex,long correctDist){
	/*long distance = Long.MAX_VALUE;
	for(int i=0;i<forwprocessedVertices.size();i++){
		int temp = forwprocessedVertices.get(i);
		if(reverseGraph[temp].distance != Long.MAX_VALUE && distance > graph[temp].distance + reverseGraph[temp].distance){
			distance = graph[temp].distance + reverseGraph[temp].distance;
		}
	}

	for(int i=0;i<revprocessedVertices.size();i++){
		int temp = revprocessedVertices.get(i);
		if(graph[temp].distance != Long.MAX_VALUE && distance > graph[temp].distance + reverseGraph[temp].distance){
			distance = graph[temp].distance + reverseGraph[temp].distance;
		}
	}*/
	return correctDist;
    }
		

	
		
    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
        int n = in.nextInt();
        int m = in.nextInt();

	Vertex [] graph = new Vertex[n];
	Vertex [] reverseGraph = new Vertex[n];
	
	for (int i = 0; i < n; i++) { 
            int x, y;
            x = in.nextInt();
            y = in.nextInt();
	
	    graph[i] = new Vertex(i,x,y);
	    reverseGraph[i] = new Vertex(i,x,y);
          
	}

	

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

        int q = in.nextInt();

        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;
            t = in.nextInt()-1;
            System.out.println(computeDist(graph,reverseGraph,s,t));
        }
    }
}
