/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import javax.xml.bind.annotation.adapters.HexBinaryAdapter;

import geography.GeographicPoint;
import javafx.scene.control.cell.MapValueFactory;
import util.GraphLoader;
import week2example.MazeNode;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, Vertex> mapVertices;
	private HashMap<Vertex,ArrayList<Vertex>> adjListMap;
	private HashMap<Vertex, Double> startDistancesMap;
	private HashMap<Vertex, Double> goalDistancesMap;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		mapVertices = new HashMap<GeographicPoint, Vertex> ();
		adjListMap = new HashMap<Vertex,ArrayList<Vertex>>();
		startDistancesMap = new HashMap<Vertex, Double>();
		goalDistancesMap = new HashMap<Vertex, Double>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return mapVertices.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return mapVertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int numEdges = 0;
		Set<GeographicPoint> verticesLocSet = mapVertices.keySet();
		for (GeographicPoint iter : verticesLocSet) {
			if (mapVertices.get(iter).getEdges() != null ) {
				numEdges += mapVertices.get(iter).getEdges().size();
			}
		}
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (mapVertices.get(location) != null) return false;

		Vertex newVertex = new Vertex(location);
		ArrayList<Vertex> neighbors = new ArrayList<>();
		adjListMap.put(newVertex, neighbors);
		mapVertices.put(location, newVertex);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 2
		if (from == null || to == null) throw new IllegalArgumentException("Starting point or ending point is not specified");
		if ( (mapVertices.get(from) == null) || (mapVertices.get(to) == null)) throw new IllegalArgumentException("Both starting point and ending point must be already added to the vertices list");
		if (length < 0) throw new IllegalArgumentException("Length of an edge cannot must be greater than or equal to zero");
		
		//Adding an edge in edgesList of particular vertex
		Edge newEdge = new Edge(from, to, roadName, roadType, length);
		mapVertices.get(from).addEdge(newEdge);
		
		//Adding the vertex the neighbors list, so that the edge will be reflected in adjacency list
		Vertex tempVertexFrom = mapVertices.get(from);
		Vertex tempVertexTo = mapVertices.get(to);
		(adjListMap.get(tempVertexFrom)).add(tempVertexTo);
	}
	
	//Additional methods
	private Vertex findVertex(GeographicPoint loc) {
		if (mapVertices.containsKey(loc) == false) return null;
		
		return mapVertices.get(loc);
	}
	
	private List<Vertex> getNeighborsOfVertex(Vertex vert) {
		if (vert == null) return null;
		
		return adjListMap.get(vert);
	}

	private void printGraphVertices() {
		Set<Vertex> verticesSet = adjListMap.keySet();
		for (Vertex iter : verticesSet) {
			if (adjListMap.get(iter) != null ) {
				System.out.println("Vertex:"+iter+" Neighbors:"+adjListMap.get(iter));
			}
		}
	}
	private void printGraphEdges() {
		Set<GeographicPoint> verticesLocSet = mapVertices.keySet();
		for (GeographicPoint iter : verticesLocSet) {
			if (mapVertices.get(iter).getEdges() != null ) {
				List<Edge> edgeList = mapVertices.get(iter).getEdges();
				for (Edge iterEdge : edgeList) {
					System.out.println(iterEdge);
					System.out.println("");
				};
			}
		}
	}
	
	private double calcPathLength(List<GeographicPoint> gpoints) {
		//TODO: Write a pethod using getDistanceToNeighbor method, included in vertex class
		if (gpoints.size() < 2) throw new IllegalArgumentException("Error in calcPathLength method: input list size must be greater than 1");
		double pathLength = 0;
		double dblTmp;

		for (int i = gpoints.size()-1; i > 0; i--) {
			dblTmp = mapVertices.get(gpoints.get(i)).getDistancetoNeighbor(gpoints.get(i-1));
			pathLength += dblTmp;
		}
		return pathLength;
	}
	
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) throws IllegalArgumentException {
		
		if (start == null || goal == null) {
			throw new IllegalArgumentException("Both start point and end point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false || mapVertices.containsKey(goal) == false) {
			throw new IllegalArgumentException("Both start point and end point must be added to the graph before searching path");
		}
		Vertex startVer = mapVertices.get(start);
		Vertex goalVer = mapVertices.get(goal);
		//List<GeographicPoint> result = new ArrayList<GeographicPoint>();
		HashSet<Vertex> visited = new HashSet<Vertex>();
		Queue<Vertex> toExplore = new LinkedList<Vertex>();
		HashMap<Vertex, Vertex> parentMap = new HashMap<>();
		boolean found = false;
		
		toExplore.add(startVer);
		while (toExplore.isEmpty() == false) {
			Vertex curr = toExplore.remove();
			if (curr == goalVer) {
				found = true;
				break;
			}
			List<Vertex> neighbors = getNeighborsOfVertex(curr);
			for (Vertex iter : neighbors) {
				if (!visited.contains(iter)) {
					visited.add(iter);
					parentMap.put(iter, curr);
					toExplore.add(iter);
				}
			}
		}
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		//Creates a list of geographicPoint objects based on the parentMap
		return constructPath(startVer, goalVer, parentMap);
		
		// Dummy variable for calling the search algorithms
        //Consumer<GeographicPoint> temp = (x) -> {};
        //return bfs(start, goal, temp);
	}
	
	//Goes back through the parentMap from goal to finish, creating a list of geographicPoints
	private List<GeographicPoint> constructPath(Vertex start, Vertex goal,
			HashMap<Vertex, Vertex> parentMap)
	{
		LinkedList<GeographicPoint> path = new LinkedList<>();
		Vertex curr = goal;
		while (curr != start) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		path.addFirst(start.getLocation());
		return path;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		if (start == null || goal == null) {
			throw new IllegalArgumentException("Both start point and end point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false || mapVertices.containsKey(goal) == false) {
			throw new IllegalArgumentException("Both start point and end point must be added to the graph before searching path");
		}
		Vertex startVer = mapVertices.get(start);
		Vertex goalVer = mapVertices.get(goal);

		HashSet<Vertex> visited = new HashSet<Vertex>();
		Queue<Vertex> toExplore = new LinkedList<Vertex>();
		HashMap<Vertex, Vertex> parentMap = new HashMap<>();
		boolean found = false;
		
		toExplore.add(startVer);
		while (toExplore.isEmpty() == false) {
			Vertex curr = toExplore.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			
			if (curr == goalVer) {
				found = true;
				break;
			}
			List<Vertex> neighbors = getNeighborsOfVertex(curr);
			for (Vertex iter : neighbors) {
				if (!visited.contains(iter)) {
					
					visited.add(iter);
					parentMap.put(iter, curr);
					toExplore.add(iter);
				}
			}
		}
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		

		return constructPath(startVer, goalVer, parentMap);
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		//Validity check
		if (start == null || goal == null) {
			throw new IllegalArgumentException("Both start point and end point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false || mapVertices.containsKey(goal) == false) {
			throw new IllegalArgumentException("Both start point and end point must be added to the graph before searching path");
		}
		
		//Initializing vertices in the graph, along with their distances
		Vertex startVer = mapVertices.get(start);
		Vertex goalVer = mapVertices.get(goal);
		for (Vertex vrtx : mapVertices.values()) {
			vrtx.setDistanceStart(Double.POSITIVE_INFINITY);
		}
		startVer.setDistanceStart(0.0);
		
		//Additional structures that will be needed
		HashSet<Vertex> visited = new HashSet<Vertex>();
		PriorityQueue<Vertex> toExplore = new PriorityQueue<Vertex>(getNumVertices(), VertexSDistComparator);
		HashMap<Vertex, Vertex> parentMap = new HashMap<Vertex, Vertex>();
		
		boolean found = false;
		toExplore.add(startVer);

		while (toExplore.isEmpty() == false) {
			Vertex curr = toExplore.poll();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			// For testing purposes:
			//System.out.println("Djikstra visiting: "+curr+"");
			if (!visited.contains(curr)) {
				visited.add(curr);
				if (curr == goalVer) {
					found = true;
					break;
				}
				List<Vertex> neighbors = getNeighborsOfVertex(curr);
				
				for (Vertex iter : neighbors) {
					if (!visited.contains(iter)) {
						double prevDistance = iter.getDistanceStart();
						double actualDistance = iter.getLocation().distance(curr.getLocation()) + curr.getDistanceStart();
						if (actualDistance < prevDistance) {
							
							iter.setDistanceStart(actualDistance);
							parentMap.put(iter, curr);
							toExplore.add(iter);
						}
						
					}
				}
				
				
			}		

		}
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		
		return constructPath(startVer, goalVer, parentMap);
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		if (start == null || goal == null) {
			throw new IllegalArgumentException("Both start point and end point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false || mapVertices.containsKey(goal) == false) {
			throw new IllegalArgumentException("Both start point and end point must be added to the graph before searching path");
		}
		//Initializing vertex in the graph
		Vertex startVer = mapVertices.get(start);
		Vertex goalVer = mapVertices.get(goal);
		for (Vertex vrtx : mapVertices.values()) {
			vrtx.setDistanceStart(Double.POSITIVE_INFINITY);
			vrtx.setDistanceGoal(vrtx.getLocation().distance(goal));
		}
		startVer.setDistanceStart(0.0);
		
		HashSet<Vertex> visited = new HashSet<Vertex>();
		PriorityQueue<Vertex> toExplore = new PriorityQueue<Vertex>(getNumVertices(), VertexDistancesComparator);

		HashMap<Vertex, Vertex> parentMap = new HashMap<Vertex, Vertex>();
		boolean found = false;
		toExplore.add(startVer);

		while (toExplore.isEmpty() == false) {
			Vertex curr = toExplore.poll();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			// For testing purposes:
			//System.out.println("A* visiting: "+curr.toString());
			
			if (!visited.contains(curr)) {
				visited.add(curr);
				if (curr == goalVer) {
					found = true;
					break;
				}
				List<Vertex> neighbors = getNeighborsOfVertex(curr);
				
				for (Vertex iter : neighbors) {
					if (!visited.contains(iter)) {
						double prevDistance = iter.getDistanceStart();
						double actualDistance = iter.getLocation().distance(curr.getLocation()) + curr.getDistanceStart();
						if (actualDistance < prevDistance) {
							iter.setDistanceStart(actualDistance);
							parentMap.put(iter, curr);
							toExplore.add(iter);
						}
						
					}
				}
				
				
			}		

		}
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		
		return constructPath(startVer, goalVer, parentMap);
	
	}
	
	
	private List<GeographicPoint> aStarSearchSpec(GeographicPoint start, 
			 GeographicPoint goal, HashSet<Vertex> visited) {
		// TODO: Implement this method in WEEK 3

		if (start == null || goal == null) {
		throw new IllegalArgumentException("Both start point and end point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false || mapVertices.containsKey(goal) == false) {
		throw new IllegalArgumentException("Both start point and end point must be added to the graph before searching path");
		}
		//Initializing vertex in the graph
		Vertex startVer = mapVertices.get(start);
		Vertex goalVer = mapVertices.get(goal);
		for (Vertex vrtx : mapVertices.values()) {
			vrtx.setDistanceStart(Double.POSITIVE_INFINITY);
			vrtx.setDistanceGoal(vrtx.getLocation().distance(goal));
		}
		startVer.setDistanceStart(0.0);
		
		//HashSet<Vertex> visited = new HashSet<Vertex>();
		PriorityQueue<Vertex> toExplore = new PriorityQueue<Vertex>(getNumVertices(), VertexDistancesComparator);
		
		HashMap<Vertex, Vertex> parentMap = new HashMap<Vertex, Vertex>();
		boolean found = false;
		toExplore.add(startVer);
		
		while (toExplore.isEmpty() == false) {
			Vertex curr = toExplore.poll();
				
			if (!visited.contains(curr)) {
				visited.add(curr);
				if (curr == goalVer) {
					found = true;
					break;
				}
				List<Vertex> neighbors = getNeighborsOfVertex(curr);
				
				for (Vertex iter : neighbors) {
					if (!visited.contains(iter)) {
						double prevDistance = iter.getDistanceStart();
						double actualDistance = iter.getLocation().distance(curr.getLocation()) + curr.getDistanceStart();
						if (actualDistance < prevDistance) {
							iter.setDistanceStart(actualDistance);
							parentMap.put(iter, curr);
							toExplore.add(iter);
						}
					}
				}
				
				
			}		
			
		}
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		
		return constructPath(startVer, goalVer, parentMap);

	}
	
	
	/** Find the path from start visiting all specified points, using Greedy Search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the
	 *   path from start through all specified points.
	 */
	public List<GeographicPoint> tspGreedySearch(GeographicPoint start, List<GeographicPoint> stops) {
		if (start == null) {
			throw new IllegalArgumentException("Error in tspGreedySearch method: Start point must be specified to search the path.");
		}
		if (mapVertices.containsKey(start) == false) {
			throw new IllegalArgumentException("Error in tspGreedySearch method: Start point must be added to the graph before searching path");
		}
		List<GeographicPoint> bestPath = new LinkedList<>();;
		GeographicPoint curr = start;
		//This list will be used to track, which nodes were visited so we can avoid these
		HashSet<Vertex> allVisited = new HashSet<Vertex>();
		HashSet<Vertex> tempVisited = new HashSet<Vertex>();
		Queue<GeographicPoint> toVisit = new LinkedList<GeographicPoint>();
		List<GeographicPoint> tempPath = new LinkedList<>();
		double tempLength = 0;
		List<GeographicPoint> closestPath = new LinkedList<>();;
		bestPath.add(start);
		
		for (GeographicPoint gp : stops) {
			//To test if all the geographic points from the list are already added to the graph
			if (mapVertices.containsKey(gp) == false) throw new IllegalArgumentException("All the stopping points must be added to the graph before searching path");
			//Creating a list of vertices to be visited from geographic point list
			toVisit.add( gp );
		}
		while (toVisit.isEmpty() == false) {
			double closestLength = Double.POSITIVE_INFINITY;
			//This for loop finds the closest geographic point to the current from the toVisit length
			for (GeographicPoint gpIter : toVisit) { //W tej chwili to jest bez sensu, powinniœmy iterowaæ po geographicPoint, nie po Vertex
				tempVisited = new HashSet<Vertex>();
				tempVisited.addAll(allVisited);
				tempPath = aStarSearchSpec(curr, gpIter, tempVisited);
				tempLength = calcPathLength(tempPath);
				if (tempLength < closestLength) {
					closestLength = tempLength;
					closestPath = tempPath;
				}
			}
			//Add found path to the closest geographic point (as a gp list) to the bestPath
				//Just removing the first element from the list, to avoid duplicates
			if (closestPath.get(0) != start) closestPath.remove(0);
			bestPath.addAll(closestPath);
			//Set the closest geographic point as current
			curr = closestPath.get( closestPath.size()-1 );
			//Make sure that curr geographic point will not be visited again
			//allVisited contains points that were already added to the bestPath
			//based on allVisited, tempVisited is created. In each iteration tempVisited is 'reseted' so in each aStarSearch are included only points already added to allVisited
			toVisit.remove(curr);
			for (GeographicPoint gpIter : bestPath) {
				if (gpIter != curr)	allVisited.add( mapVertices.get(gpIter) );
			}
		}
		return bestPath;
			
		
	}
	
	Comparator<Vertex> VertexSDistComparator = new Comparator<Vertex>() {

		@Override
		public int compare(Vertex v1, Vertex v2) {
			if (v1.getDistanceStart() > v2.getDistanceStart()) {
				return 1;
			} else if (v1.getDistanceStart() <= v2.getDistanceStart()) {
				return -1;
			}
			return 0;
		}
		
	};
	Comparator<Vertex> VertexDistancesComparator = new Comparator<Vertex>() {

		@Override
		public int compare(Vertex v1, Vertex v2) {
			double vCost1 = v1.getDistanceStart() + v1.getDistanceGoal();
			double vCost2 = v2.getDistanceStart() + v2.getDistanceGoal();
			if (vCost1 > vCost2) {
				return 1;
			} else if (vCost1 <= vCost2) {
				return -1;
			}
			return 0;
		}
		
	};
	

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		//System.out.println("Showing number of vertices of current graph: "+ firstMap.getNumVertices() );
		//System.out.println("Showing number of (directed) edges in current graph: "+ firstMap.getNumEdges() );
		//firstMap.printGraphEdges();
		

		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		List<GeographicPoint> stops = new LinkedList<GeographicPoint>();
		stops = Arrays.asList(new GeographicPoint(7.0, 3.0), new GeographicPoint(4.0, 0.0), new GeographicPoint(8.0, -1.0));
		List<GeographicPoint> testroute3 = simpleTestMap.tspGreedySearch(testStart, stops);
		for (GeographicPoint gpIter : testroute3) {
			System.out.println(gpIter);
		}
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		stops = Arrays.asList(new GeographicPoint(32.86132, -117.2227588), new GeographicPoint(32.860832, -117.218933) );
		GeographicPoint start = new GeographicPoint(32.860416, -117.221878);
		List<GeographicPoint> testroute4 = testMap.tspGreedySearch(start, stops);
		for (GeographicPoint gpIter : testroute4) {
			System.out.println(gpIter);
		}
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
//		
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//
//		
//		
//		/* Use this code in Week 3 End of Week Quiz */
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//		
//		
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
	}
	
}
