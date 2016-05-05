package cz.agents.alite.trajectorytools.planner;

import java.util.ArrayList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

public class SingleVertexPlannedPath<V, E> implements
        PlannedPath<V, E> {
    private final Graph<V,E> graph;
    private final V vertex;

    public SingleVertexPlannedPath(Graph<V,E> graph, V vertex) {
        this.graph = graph;
        this.vertex = vertex;
    }

    @Override
    public Graph<V, E> getGraph() {
        return graph;
    }

    @Override
    public V getStartVertex() {
        return vertex;
    }

    @Override
    public V getEndVertex() {
        return vertex;
    }

    @Override
    public List<E> getEdgeList() {
        return new ArrayList<E>();
    }

    @Override
    public double getWeight() {
        return 0;
    }

    @Override
    public List<E> getPathEdgeList() {
        return new ArrayList<E>();
    }

    @Override
    public GraphPath<V, E> getPath() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getPathLength() {
        return 0;
    }
    
    @Override
    public boolean equals(Object obj) {
    	if (obj instanceof SingleVertexPlannedPath) {
    		SingleVertexPlannedPath<?, ?> path = (SingleVertexPlannedPath<?, ?>) obj;
    		return vertex.equals(path.vertex);
    	} else {
    		return false;
    	}
    }
    
    @Override
    public int hashCode() {
    	return vertex.hashCode();
    }
}