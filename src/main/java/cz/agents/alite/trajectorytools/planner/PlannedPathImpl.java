package cz.agents.alite.trajectorytools.planner;

import java.util.Iterator;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.GraphPathImpl;

public class PlannedPathImpl <V, E> extends GraphPathImpl<V, E> implements PlannedPath<V, E> {
    
    //~ Methods ----------------------------------------------------------------

    public PlannedPathImpl(Graph<V, E> graph, V startVertex, V endVertex,
            List<E> edgeList, double weight) {
        super(graph, startVertex, endVertex, edgeList, weight);
    }

    public PlannedPathImpl(Graph<V, E> graph, List<E> edges) {
        this(graph, getStartVertex(graph, edges), getEndVertex(graph, edges), edges, getWeight(graph, edges));
    }

    private static <V, E> V getStartVertex(Graph<V, E> graph, List<E> edges) {
        E edge = edges.get(0);
        return graph.getEdgeSource(edge);
    }

    private static <V, E> V getEndVertex(Graph<V, E> graph, List<E> edges) {
        E edge = edges.get(edges.size() - 1);
        return graph.getEdgeTarget(edge);
    }
    
    private static <V, E> double getWeight(Graph<V, E> graph, List<E> edges) {
        double weight = 0;
        for (E edge : edges) {
            weight += graph.getEdgeWeight(edge);
        }
        return weight;
    }

    /**
     * Return the edges making up the path found.
     *
     * @return List of Edges, or null if no path exists
     */
    @Override
    public List<E> getPathEdgeList()
    {
        return getEdgeList();
    }

    /**
     * Return the path found.
     *
     * @return path representation, or null if no path exists
     */
    @Override
    public GraphPath<V, E> getPath()
    {
        return this;
    }

    /**
     * Return the length of the path found.
     *
     * @return path length, or Double.POSITIVE_INFINITY if no path exists
     */
    @Override
    public double getPathLength()
    {
        return getPath().getWeight();
    }
    
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof PlannedPath<?, ?>) {
            @SuppressWarnings("unchecked")
            PlannedPath<V, E> path = (PlannedPath<V, E>) obj;
            if (getPathLength() != path.getPathLength()) {
                return false;
            } else {
            	Iterator<E> pathIt = path.getEdgeList().iterator(); 
            	Iterator<E> it = getEdgeList().iterator();
            	while (it.hasNext()) {
            		E pathEdge = pathIt.next();
            		E edge = it.next();
            		
            		if (!getGraph().getEdgeSource(edge).equals(getGraph().getEdgeSource(pathEdge)) 
        				|| !getGraph().getEdgeTarget(edge).equals(getGraph().getEdgeTarget(pathEdge))) {
            			return false;
            		}
            	}
                return true;
            }
        } else {
            return false;
        }
    }
    
    @Override
    public int hashCode() {
        return toString().hashCode();
    }
}
