//Program to implement Contraction Hierarchies Algorithm.

import java.util.Scanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.*;
import java.util.PriorityQueue;
import java.util.Comparator;

public class DistPreprocessSmall{

   static class Distance{
	//Ids are made so that we dont have to reinitialize everytime the distance value to infinity.

	int contractId;		//id for the vertex that is going to be contracted.
	int sourceId;           //it contains the id of vertex for which we will apply dijkstra while contracting.
	
	long distance; 		//stores the value of distance while contracting.

	//used in query time for bidirectional dijkstra algo
	int forwqueryId; 	//for forward search.
	int revqueryId; 	//for backward search.

	long queryDist; 	//for forward distance.
	long revDistance; 	//for backward distance.

	public Distance(){
		this.contractId = -1;
		this.sourceId=-1;
		
		this.forwqueryId=-1;
		this.revqueryId=-1;
		
		this.distance = Integer.MAX_VALUE;		

		this.revDistance = Integer.MAX_VALUE;
		this.queryDist = Integer.MAX_VALUE;
	}
    }
  
    //in this ids are made for the same reason, to not have to reinitialize processed variable for every query in bidirectional dijkstra.
    static class Processed{
	boolean forwProcessed; 	//is processed in forward search.
	boolean revProcessed; 	//is processed in backward search.
	int forwqueryId; 	//id for forward search.
	int revqueryId; 	//id for backward search.

	public Processed(){
		this.forwqueryId=-1;
		this.revqueryId=-1;
	}
    }
	
    //class for Vertex of a graph.
    static class Vertex{
	int vertexNum;			//id of the vertex.
	ArrayList<Integer> inEdges; 	//list of incoming edges to this vertex.
	ArrayList<Long> inECost;	//list of incoming edges cost or distance.	
	ArrayList<Integer> outEdges; 	//list of outgoing edges from this vertex.
	ArrayList<Long> outECost;	//list of out edges cost or distance.
	
	int orderPos; 			//position of vertex in nodeOrderingQueue.
	
	boolean contracted; 		//to check if vertex is contracted
	
	Distance distance;
	Processed processed;
	
	//parameters for computing importance according to which we will contract the vertices. Vertex wih least importance wil be contracted first.
	int edgeDiff; 			//egdediff = sE - inE - outE. (sE=inE*outE , i.e number of shortcuts that we may have to add.)
	long delNeighbors; 		//number of contracted neighbors.
	int shortcutCover; 		//number of shortcuts to be introduced if this vertex is contracted.

	long importance; 		//total importance = edgediff + shortcutcover + delneighbors.

	public Vertex(){
	}
	
	public Vertex(int vertexNum){
		this.vertexNum=vertexNum;
		this.inEdges = new ArrayList<Integer>();
		this.outEdges = new ArrayList<Integer>();
		this.inECost = new ArrayList<Long>();
		this.outECost = new ArrayList<Long>();
		this.distance = new Distance();
		this.processed = new Processed();
		this.delNeighbors = 0;
		this.contracted=false;
	}
    }
    
    
    //priorityQueue (based on min heap) dealing with importance parameter.
    public static class PQIMPcomparator implements Comparator<Vertex>{
	public int compare(Vertex node1, Vertex node2){
		if(node1.importance > node2.importance){
			return 1;
		}
		if(node1.importance < node2.importance){
			return -1;
		}
		return 0;
	}
    }

    
    //priorityQueue (min heap) dealing with distance while preprocessing time.
    static class PriorityQueueComp implements Comparator<Vertex>{
	public int compare(Vertex node1,Vertex node2){
		if(node1.distance.distance>node2.distance.distance){
			return 1;
		}
		if(node1.distance.distance<node2.distance.distance){
			return -1;
		}
		return 0;
	}
    }



    //all functions dealing with preprocessing in this class.
    static class PreProcess{
	Comparator<Vertex> comp = new PQIMPcomparator();
	PriorityQueue<Vertex> PQImp;  	//queue for importance parameter.

	Comparator<Vertex> PQcomp = new PriorityQueueComp();
	PriorityQueue<Vertex> queue;	//queue for distance parameter. 
	
	
	//calculate initial importance for all vertices.
	private void computeImportance(Vertex [] graph){
		PQImp = new PriorityQueue<Vertex>(graph.length,comp);
		for(int i=0;i<graph.length;i++){
			graph[i].edgeDiff = (graph[i].inEdges.size() * graph[i].outEdges.size()) - graph[i].inEdges.size() - graph[i].outEdges.size();
			graph[i].shortcutCover = graph[i].inEdges.size() + graph[i].outEdges.size();
			graph[i].importance = graph[i].edgeDiff*14+ graph[i].shortcutCover*25 + graph[i].delNeighbors*10;
			PQImp.add(graph[i]);
		}
	}


	//compute importance for individual vertex while processing.
	private void computeImportance(Vertex [] graph, Vertex vertex){
		vertex.edgeDiff = (vertex.inEdges.size() * vertex.outEdges.size()) - vertex.inEdges.size() - vertex.outEdges.size();
		vertex.shortcutCover = vertex.inEdges.size() + vertex.outEdges.size();
		vertex.importance = vertex.edgeDiff*14 + vertex.shortcutCover*25 + vertex.delNeighbors*10;
	}
	
	
	//function that will pre-process the graph.
	private int [] preProcess(Vertex [] graph){
		int [] nodeOrdering = new int[graph.length];	//contains the vertices in the order they are contracted.
		int extractNum=0; 				//stores the number of vertices that are contracted.
		
		while(PQImp.size()!=0){
			Vertex vertex = (Vertex)PQImp.poll();
			computeImportance(graph,vertex);	//recompute importance before contracting the vertex.
			
			//if the vertex's recomputed importance is still minimum then contract it.
			if(PQImp.size()!=0 && vertex.importance > PQImp.peek().importance){
				PQImp.add(vertex);
				continue;
			}
		
			nodeOrdering[extractNum] = vertex.vertexNum;
			vertex.orderPos = extractNum; 
			extractNum = extractNum + 1;
			
			//contraction part.
			contractNode(graph,vertex,extractNum-1);
		}
		return nodeOrdering;
	}

	
	//update the neighbors of the contracted vertex that this vertex is contracted.
	private void calNeighbors(Vertex [] graph,ArrayList<Integer> inEdges, ArrayList<Integer> outEdges){
		for(int i=0;i<inEdges.size();i++){
			int temp =inEdges.get(i);
			graph[temp].delNeighbors++;
		}

		for(int i=0;i<outEdges.size();i++){
			int temp =outEdges.get(i);
			graph[temp].delNeighbors++;
		}
	}


	//function to contract the node.
	private void contractNode(Vertex [] graph, Vertex vertex, int contractId){
		ArrayList<Integer> inEdges = vertex.inEdges;
		ArrayList<Long> inECost = vertex.inECost;
		ArrayList<Integer> outEdges = vertex.outEdges;
		ArrayList<Long> outECost = vertex.outECost;
		
		vertex.contracted=true;
		
		long inMax = 0;						//stores the max distance out of uncontracted inVertices of the given vertex.
		long outMax =0;						//stores the max distance out of uncontracted outVertices of the given vertex.

		calNeighbors(graph,vertex.inEdges,vertex.outEdges);	//update the given vertex's neighbors about that the given vertex is contracted.
			
		for(int i=0; i<inECost.size();i++){
			if(graph[inEdges.get(i)].contracted){
				continue;
			}
			if(inMax<inECost.get(i)){
				inMax = inECost.get(i);
			}
		}

		for(int i=0; i<outECost.size();i++){
			if(graph[outEdges.get(i)].contracted){
				continue;
			}
			if(outMax<outECost.get(i)){
				outMax = outECost.get(i);
			}
		}

		long max = inMax+outMax; 				//total max distance.
		
		for(int i=0;i<inEdges.size();i++){
			int inVertex = inEdges.get(i);
			if(graph[inVertex].contracted){
				continue;
			}
			long incost = inECost.get(i);
			
			dijkstra(graph,inVertex,max,contractId,i); 	//finds the shortest distances from the inVertex to all the outVertices.

			//this code adds shortcuts.
			for(int j=0;j<outEdges.size();j++){
				int outVertex = outEdges.get(j);
				long outcost = outECost.get(j);
				if(graph[outVertex].contracted){
					continue;
				}
				if(graph[outVertex].distance.contractId!=contractId || graph[outVertex].distance.sourceId!=i || graph[outVertex].distance.distance>incost+outcost){
					graph[inVertex].outEdges.add(outVertex);
					graph[inVertex].outECost.add(incost+outcost);
					graph[outVertex].inEdges.add(inVertex);
					graph[outVertex].inECost.add(incost+outcost);
				}
			}
		}
	}

	
	//dijkstra function implemented.
	private void dijkstra(Vertex [] graph, int source, long maxcost, int contractId,int sourceId){
		queue = new PriorityQueue<Vertex>(graph.length,PQcomp);
		
		graph[source].distance.distance = 0;
		graph[source].distance.contractId=contractId;
		graph[source].distance.sourceId = sourceId;
		
		queue.clear();
		queue.add(graph[source]);
		
		int i=0;
		while(queue.size()!=0){
			Vertex vertex = (Vertex)queue.poll();
			if(i>3 || vertex.distance.distance > maxcost){
				return;
			}
			relaxEdges(graph,vertex.vertexNum,contractId,queue,sourceId);
		}
	}
	
	//function to relax outgoing edges. 
	private void relaxEdges(Vertex [] graph,int vertex,int contractId, PriorityQueue queue,int sourceId){
		ArrayList<Integer> vertexList = graph[vertex].outEdges;
		ArrayList<Long> costList = graph[vertex].outECost;
		
		for(int i=0;i<vertexList.size();i++){
			int temp = vertexList.get(i);
			long cost = costList.get(i);
			if(graph[temp].contracted){
				continue;
			}
			if(checkId(graph,vertex,temp) || graph[temp].distance.distance > graph[vertex].distance.distance + cost){
				graph[temp].distance.distance = graph[vertex].distance.distance + cost;
				graph[temp].distance.contractId = contractId;
				graph[temp].distance.sourceId = sourceId;
				
				queue.remove(graph[temp]);
				queue.add(graph[temp]);
			}
		}
	}

	//compare the ids whether id of source to target is same if not then consider the target vertex distance=infinity.
	private boolean checkId(Vertex [] graph,int source,int target){
		if(graph[source].distance.contractId != graph[target].distance.contractId || graph[source].distance.sourceId != graph[target].distance.sourceId){
			return true;
		}
		return false;
	}

	//main function of this class.
	public int [] processing(Vertex [] graph){
		computeImportance(graph);		//find initial importance by traversing all vertices.
		int [] nodeOrdering = preProcess(graph);
		return nodeOrdering;
	}
    }


			
     
    //priorityQueue(min heap) for bidirectional dijkstra algorithms.(for forward search)
    public static class forwComparator implements Comparator<Vertex>{
	public int compare(Vertex vertex1, Vertex vertex2){
		if(vertex1.distance.queryDist>vertex2.distance.queryDist){
			return 1;
		}
		if(vertex1.distance.queryDist<vertex2.distance.queryDist){
			return -1;
		}
		return 0;
	}
    }
   


    //priorityQueue(min heap) for bidirectional dijkstra algorithms.(for backward search)
    public static class revComparator implements Comparator<Vertex>{
	public int compare(Vertex vertex1, Vertex vertex2){
		if( vertex1.distance.revDistance>vertex2.distance.revDistance){
			return 1;
		}
		if(vertex1.distance.revDistance<vertex2.distance.revDistance){
			return -1;
		}
		return 0;
	}
    }

    

    //class for bidirectional dijstra search.
    static class BidirectionalDijkstra{
	Comparator<Vertex> forwComp = new forwComparator();
    	Comparator<Vertex> revComp = new revComparator();
    	PriorityQueue<Vertex> forwQ;
    	PriorityQueue<Vertex> revQ;
	
 	//main function that will compute distances.
	public long computeDist(Vertex [] graph, int source, int target, int queryID , int [] nodeOrdering){
		graph[source].distance.queryDist = 0;
		graph[source].distance.forwqueryId = queryID;
		graph[source].processed.forwqueryId = queryID;

		graph[target].distance.revDistance = 0;
		graph[target].distance.revqueryId = queryID;
		graph[target].processed.revqueryId = queryID;

		forwQ = new PriorityQueue<Vertex>(graph.length,forwComp);
		revQ = new PriorityQueue<Vertex>(graph.length,revComp);
		
		forwQ.add(graph[source]);
		revQ.add(graph[target]);

		long estimate = Long.MAX_VALUE;
		
		while(forwQ.size()!=0 || revQ.size()!=0){
			if(forwQ.size()!=0){
				Vertex vertex1 = (Vertex)forwQ.poll();
				if(vertex1.distance.queryDist<=estimate){
					relaxEdges(graph,vertex1.vertexNum,"f",nodeOrdering,queryID);
				}
				if(vertex1.processed.revqueryId == queryID && vertex1.processed.revProcessed){
					if(vertex1.distance.queryDist + vertex1.distance.revDistance < estimate){
						estimate = vertex1.distance.queryDist + vertex1.distance.revDistance;
					}
				}
			}
			
			if(revQ.size()!=0){
				Vertex vertex2 = (Vertex)revQ.poll();
				if(vertex2.distance.revDistance <= estimate){
					relaxEdges(graph,vertex2.vertexNum,"r",nodeOrdering,queryID);
				}
				if(vertex2.processed.forwqueryId == queryID && vertex2.processed.forwProcessed){
					if(vertex2.distance.revDistance + vertex2.distance.queryDist < estimate){
						estimate = vertex2.distance.queryDist + vertex2.distance.revDistance;
					}
				}
			}
		}

		if(estimate==Long.MAX_VALUE){
			return -1;
		}
		return estimate;
	}



	//function to relax edges.(according to the direction forward or backward)
	private void relaxEdges(Vertex [] graph, int vertex,String str,int [] nodeOrdering, int queryId){
		if(str == "f"){
			ArrayList<Integer> vertexList = graph[vertex].outEdges;
			ArrayList<Long> costList = graph[vertex].outECost;
			graph[vertex].processed.forwProcessed=true;
			graph[vertex].processed.forwqueryId = queryId;
			
			for(int i=0;i<vertexList.size();i++){
				int temp = vertexList.get(i);
				long cost = costList.get(i);
				if(graph[vertex].orderPos < graph[temp].orderPos){
					if(graph[vertex].distance.forwqueryId != graph[temp].distance.forwqueryId || graph[temp].distance.queryDist > graph[vertex].distance.queryDist + cost){
						graph[temp].distance.forwqueryId = graph[vertex].distance.forwqueryId;
						graph[temp].distance.queryDist = graph[vertex].distance.queryDist + cost;
						
						forwQ.remove(graph[temp]);
						forwQ.add(graph[temp]);
					}
				}
			}
		}
		else{
			ArrayList<Integer> vertexList = graph[vertex].inEdges;
			ArrayList<Long> costList = graph[vertex].inECost;
			graph[vertex].processed.revProcessed = true;
			graph[vertex].processed.revqueryId = queryId;
			
			for(int i=0;i<vertexList.size();i++){
				int temp = vertexList.get(i);
				long cost = costList.get(i);
				
				if(graph[vertex].orderPos < graph[temp].orderPos){
					if(graph[vertex].distance.revqueryId != graph[temp].distance.revqueryId || graph[temp].distance.revDistance > graph[vertex].distance.revDistance + cost){
						graph[temp].distance.revqueryId = graph[vertex].distance.revqueryId;
						graph[temp].distance.revDistance = graph[vertex].distance.revDistance + cost;
						
						revQ.remove(graph[temp]);
						revQ.add(graph[temp]);
					}
				}
			}
		}
	}
		
    }

    //main function to run the program.
    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
        int n = in.nextInt();	//number of vertices in the graph.
        int m = in.nextInt();	//number of edges in the graph.

	Vertex vertex = new Vertex();
	Vertex [] graph = new Vertex[n];
	
	//initialize the graph.
	for(int i=0;i<n;i++){
		graph[i] = new Vertex(i);
	}

	//get edges
        for (int i = 0; i < m; i++) {
            int x, y;
            Long c;
            x = in.nextInt()-1;
            y = in.nextInt()-1;
            c = in.nextLong();
            
	    graph[x].outEdges.add(y);
	    graph[x].outECost.add(c);
	    graph[y].inEdges.add(x);
	    graph[y].inECost.add(c);
	}
	
	//preprocessing stage.
	PreProcess process = new PreProcess();
        int [] nodeOrdering = process.processing(graph);
       
	System.out.println("Ready");
	
	//acutal distance computation stage.
	BidirectionalDijkstra bd = new BidirectionalDijkstra();

        int t = in.nextInt();

        for (int i = 0; i < t; i++) {
            int u, v;
            u = in.nextInt()-1;
            v = in.nextInt()-1;
            System.out.println(bd.computeDist(graph,u,v,i,nodeOrdering));
        }
    }
}