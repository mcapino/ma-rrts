package cz.agents.alite.trajectorytools.graph.spatial;

import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;

@SuppressWarnings("serial")
public class TimeWeightedSpatialManeuverGraph<V,E extends SpatialManeuver> extends GraphDelegator<V, E> {
	
	public TimeWeightedSpatialManeuverGraph(Graph<V, E> g) {
		super(g);
	}

	@Override
	public double getEdgeWeight(SpatialManeuver e) {
		return e.getDuration();
	}
}
