package roadgraph;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class Vertex {
	private GeographicPoint location;
	private List<Edge> neighborEdges;
	private double distanceStart;
	private double distanceGoal;
	
	Vertex(GeographicPoint location) {
		this.location = location;
		neighborEdges = new ArrayList<Edge>();
		this.distanceStart = 0;
		this.distanceGoal = 0;
	}

	GeographicPoint getLocation() {
		return location;
	}
	void setLocation(GeographicPoint newLocation) {
		location = newLocation;
	}
	
	public boolean addEdge(Edge newEdge) {
		if (neighborEdges.contains(newEdge)) {return false;}
		
		neighborEdges.add(newEdge);
		return true;
	}
	public List<Edge> getEdges() {
		return neighborEdges;
	}
	void setDistanceStart(double distStart) {
		distanceStart = distStart;
	}
	void setDistanceGoal(double distGoal) {
		distanceGoal = distGoal;
	}
	double getDistanceStart() {
		return distanceStart;
	}
	double getDistanceGoal() {
		return distanceGoal;
	}
	double getDistancetoNeighbor(GeographicPoint gp) {
		if (gp == null) return (Double) null;
		for (Edge tmpEdge : getEdges()) {
			GeographicPoint tmpgp1 = tmpEdge.getEndPoint();
			if (tmpgp1.getX() == gp.getX() && tmpgp1.getY() == gp.getY()) {
				return tmpEdge.getRoadLength();
			}
		}
		return (Double) null;
	}
	
	public String toString() {
		String neighbors = "";
		for (Edge edge : neighborEdges) neighbors += edge.getLabel()+", ";
		return ("[Location: "+this.location+" Edges: "+neighbors+"]");
	}
	
}
