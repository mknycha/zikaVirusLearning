package roadgraph;

import geography.GeographicPoint;

public class Edge {
	private String label;
	private GeographicPoint start;
	private GeographicPoint end;
	private String roadType;
	private double length;
	
	Edge(GeographicPoint startLoc, GeographicPoint endLoc, String edgeLabel, String roadType, double length) {
		start = startLoc;
		end = endLoc;
		label = edgeLabel;
		this.roadType = roadType;
		this.length = length;
	}
	
	Edge(GeographicPoint startLoc, GeographicPoint endLoc, String edgeLabel) {
		start = startLoc;
		end = endLoc;
		label = edgeLabel;
		this.roadType = null;
		this.length = 0;
	}
	
	public String getLabel() {
		return label;
	}
	public GeographicPoint getStartPoint() {
		return start;
	}
	public GeographicPoint getEndPoint() {
		return end;
	}
	public double getRoadLength() {
		return length;
	}
	
	public String toString() {
		return "Start location: "+start+" \nEnd location: "+end+" \nLabel: "+label+" \nRoad type: "+roadType+" \nLength: "+length;
	}
	
	
}
