import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Comparator;

public class FriendSuggestion {

    static class Distance{
	int queryId;
	int distance;

	public Distance(){
		this.queryId=-1;
		this.distance = Integer.MAX_VALUE;
	}
    }

    static class Processed{
	int queryId;
	boolean processed;

	public Processed(){
		this.queryId=-1;
		this.processed=false;
	}
    }

    static class Vertex{
	int vertexNum;
	ArrayList<Integer> adjList;
	ArrayList<Integer> costList;
	//Distance distance;
	//Processed processed;

	int queuePos;
	long dist;
	boolean processed;

	public Vertex(){
	}

	public Vertex(int vertexNum){
		this.vertexNum=vertexNum;
		this.adjList = new ArrayList<Integer>();
		this.costList = new ArrayList<Integer>();
		//this.distance = new Distance();
		//this.processed = new Processed();
	}

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

   /* static class QueueComp implements Comparator<Vertex>{
	public int compare(Vertex vertex1, Vertex vertex2){
		if(vertex1.distance.distance>vertex2.distance.distance){
			return 1;
		}

		if(vertex1.distance.distance<vertex2.distance.distance){
			return -1;
		}
		return 0;
	}
    }*/

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
	
	public void changePriority(Vertex [] graph, int [] priorityQ, int index){
		if((index-1)/2 > -1 && graph[priorityQ[index]].dist < graph[priorityQ[(index-1)/2]].dist){
			swap(graph,priorityQ,index,(index-1)/2);
			changePriority(graph,priorityQ,(index-1)/2);
		}
	}
    }
   
    private static void relaxEdges(Vertex [] graph, int vertex, int [] priorityQ, PriorityQueue queue,int queryId){
	ArrayList<Integer> vertexList = graph[vertex].adjList;
	ArrayList<Integer> costList = graph[vertex].costList;
	graph[vertex].processed = true;
	//graph[vertex].processed.queryId = queryId;
	
	
	
	for(int i=0;i<vertexList.size();i++){
		int temp = vertexList.get(i);
		int cost = costList.get(i);
		/*if(graph[temp].distance.queryId != graph[vertex].distance.queryId || graph[temp].distance.distance>graph[vertex].distance.distance + cost){
			graph[temp].distance.distance = graph[vertex].distance.distance + cost;	
			//graph[temp].distance.queryId = queryId ;	
			queue.changePriority(graph,priorityQ,graph[temp].queuePos);
			//queue.remove(graph[temp]);
			//queue.add(graph[temp]);
		}*/
		if(graph[temp].dist>graph[vertex].dist + cost){
			graph[temp].dist = graph[vertex].dist + cost;	
			//graph[temp].distance.queryId = queryId ;	
			queue.changePriority(graph,priorityQ,graph[temp].queuePos);
			//queue.remove(graph[temp]);
			//queue.add(graph[temp]);
		}
	}
    }

    //static QueueComp comp = new QueueComp();
    //static PriorityQueue<Vertex> forwQ = new PriorityQueue<Vertex>(comp); 		
    //static PriorityQueue<Vertex> revQ = new PriorityQueue<Vertex>(comp);
   
    public static long computeDistance(Vertex [] graph, Vertex [] reverseGraph, int s, int t,int queryId){
	//forwQ.clear();
	//revQ.clear();
	
	//graph[s].distance.distance = 0;
	//graph[s].distance.queryId = queryId;
	
	//reverseGraph[t].distance.distance = 0;
	//reverseGraph[t].distance.queryId = queryId;
	

	//forwQ.add(graph[s]);
	//revQ.add(reverseGraph[t]);

	PriorityQueue queue = new PriorityQueue();
	int [] forwPriorityQ = new int[graph.length];
	int [] revPriorityQ = new int[graph.length];

	Vertex vertex = new Vertex();
	vertex.createGraph(graph,reverseGraph,forwPriorityQ,revPriorityQ);

	graph[s].dist=0;
	reverseGraph[t].dist=0;
	queue.makeQueue(graph,forwPriorityQ,s,t);
	queue.makeQueue(reverseGraph,revPriorityQ,t,s);



	ArrayList<Integer> forgraphprocessedVertices = new ArrayList<Integer>();
	ArrayList<Integer> revgraphprocessedVertices = new ArrayList<Integer>();
	
	

	for(int i=0;i<graph.length;i++){
		int vertex1 = queue.extractMin(graph,forwPriorityQ,i);
		if(graph[vertex1].dist==Integer.MAX_VALUE){
			continue;
		}
		relaxEdges(graph,vertex1,forwPriorityQ,queue,queryId);
		forgraphprocessedVertices.add(vertex1);

		if(reverseGraph[vertex1].processed){
			return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
		}
		

		int revVertex = queue.extractMin(reverseGraph,revPriorityQ,i);
		if(reverseGraph[revVertex].dist==Integer.MAX_VALUE){
			continue;
		}
		relaxEdges(reverseGraph,revVertex,revPriorityQ,queue,queryId);
		revgraphprocessedVertices.add(revVertex);
		
		if(graph[revVertex].processed){
			return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
		}
		
	}
	
	/*while(forwQ.size()!=0 || revQ.size()!=0){
		
		Vertex vertex = forwQ.poll();
		if(vertex!=null){
			relaxEdges(graph,vertex.vertexNum,forwQ,queryId);
			forgraphprocessedVertices.add(vertex.vertexNum);

			if(reverseGraph[vertex.vertexNum].processed.queryId==queryId && reverseGraph[vertex.vertexNum].processed.processed){
				return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
			}
		}

		Vertex revVertex = revQ.poll();
		if(revVertex!=null){
			relaxEdges(reverseGraph,revVertex.vertexNum,revQ,queryId);
			revgraphprocessedVertices.add(revVertex.vertexNum);
		
			if(graph[revVertex.vertexNum].processed.queryId==queryId && graph[revVertex.vertexNum].processed.processed){
				return shortestPath(graph,reverseGraph,forgraphprocessedVertices,revgraphprocessedVertices,queryId);
		}
		}
		
	}*/
	return -1;
    }


    private static long shortestPath(Vertex [] graph, Vertex [] reverseGraph, ArrayList<Integer> forgraphprocessedVertices, ArrayList<Integer> revgraphprocessedVertices,int queryId){
	long distance = Integer.MAX_VALUE;
	
	for(int i=0;i<forgraphprocessedVertices.size();i++){
		int vertex = forgraphprocessedVertices.get(i);
		/*if(reverseGraph[vertex].distance.queryId!=queryId || reverseGraph[vertex].distance.distance + graph[vertex].distance.distance>=Integer.MAX_VALUE){
			continue;
		}*/
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
		/*if(graph[vertex].distance.queryId!=queryId || reverseGraph[vertex].distance.distance + graph[vertex].distance.distance>=Integer.MAX_VALUE){
			continue;
		}*/
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
    

    public static void main(String args[]) {
	Scanner in = new Scanner(System.in);
        int n = in.nextInt();
	int m = in.nextInt();

	Vertex vertex = new Vertex();
	Vertex [] graph = new Vertex[n];
	Vertex [] reverseGraph = new Vertex[n];

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

	int q = in.nextInt();

        for (int i = 0; i < q; i++) {
            int s, t;
            s = in.nextInt()-1;
            t = in.nextInt()-1;
		
	    System.out.println(computeDistance(graph,reverseGraph,s,t,i));
	}
	
    }
}
