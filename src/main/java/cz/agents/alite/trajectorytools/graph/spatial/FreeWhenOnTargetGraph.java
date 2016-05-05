package cz.agents.alite.trajectorytools.graph.spatial;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;

@SuppressWarnings("serial")
public class FreeWhenOnTargetGraph<V, E extends SpatialManeuver>
		extends GraphDelegator<V, E> implements DirectedGraph<V,E> {
	
	V targetVertex;	
	
	public FreeWhenOnTargetGraph(Graph<V, E> g, V targetVertex) {
		super(g);
		this.targetVertex = targetVertex;
	}

	public FreeWhenOnTargetGraph(Graph<V, E> g) {
		super(g);
	}

	@Override
	public double getEdgeWeight(E e) {
		V edgeSource = getEdgeSource(e);
		V edgeTarget = getEdgeTarget(e);
		if (edgeSource.equals(targetVertex) && edgeTarget.equals(targetVertex)) {
			return 0;
		} else {
			return e.getDuration();
		}
	}
}
