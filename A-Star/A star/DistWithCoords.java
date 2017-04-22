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
		
	public void makeQueue(Vertex [] graph,int [] forwpriorityQ, int source, int target){
		swap(graph, forwpriorityQ,0,source);
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
 
    public static long computeDist(Vertex [] graph, int source, int target){
	
	int [] forwpriorityQ = new int[graph.length];
	initialize(graph,forwpriorityQ,source,target);	

	PriorityQueue queue = new PriorityQueue();
	queue.makeQueue(graph, forwpriorityQ,source,target);

	for(int i=0;i<graph.length;i++){
		int vertex1 = queue.extractMin(graph,forwpriorityQ,i);
		
		if(graph[vertex1].distance == (Long.MAX_VALUE)){
			return -1;
		}
		
		if(vertex1==target){
			return graph[vertex1].distance;
		}
		
		relaxEdges(graph,forwpriorityQ,vertex1,queue);
		
	}
	return -1;
    }

    
    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
        int n = in.nextInt();
        int m = in.nextInt();

	Vertex [] graph = new Vertex[n];
	
	
	
	for (int i = 0; i < n; i++) { 
            int x, y;
            x = in.nextInt();
            y = in.nextInt();
	    graph[i] = new Vertex(i,x,y);
	    
	}

	

        for (int i = 0; i < m; i++) {
            int x, y, c;
            x = in.nextInt();
            y = in.nextInt();
            c = in.nextInt();

	    graph[x-1].adjList.add(y-1);
	    graph[x-1].costList.add(c);

	}

        int q = in.nextInt();

        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;
            t = in.nextInt()-1;
            System.out.println(computeDist(graph,s,t));
        }
    }
}
