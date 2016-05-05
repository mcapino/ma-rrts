package cz.agents.alite.trajectorytools.planner;

import java.util.List;

import org.jgrapht.GraphPath;

public interface PlannedPath<V, E> extends GraphPath<V, E> {

    /**
     * Return the edges making up the path found.
     *
     * @return List of Edges, or null if no path exists
     */
    public List<E> getPathEdgeList();

    /**
     * Return the path found.
     *
     * @return path representation, or null if no path exists
     */
    public GraphPath<V, E> getPath();

    /**
     * Return the length of the path found.
     *
     * @return path length, or Double.POSITIVE_INFINITY if no path exists
     */
    public double getPathLength();

}
