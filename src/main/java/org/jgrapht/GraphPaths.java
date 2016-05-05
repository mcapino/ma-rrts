package org.jgrapht;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.graph.GraphPathImpl;

public class GraphPaths {

    public static <V,E> GraphPath<V,E> concatenate(Graph<V,E> graph, List<GraphPath<V,E>> paths) {
        V start = null;
        V end = null;
        List<E> edges = new LinkedList<E>();
        double weight = 0.0;

        if (paths.isEmpty()){
            throw new IllegalArgumentException("The list of paths to concatenate is empty");
        }


        V lastEnd = null;
        for (GraphPath<V,E> path : paths) {
            if (lastEnd != null)  {
                if (!lastEnd.equals(path.getStartVertex())) {
                    throw new IllegalArgumentException("The paths do not align");
                }
            }

            if (!path.getGraph().equals(graph)) {
                throw new IllegalArgumentException("One of the given paths is not on the given graph");
            }

            if (start == null)  {
                start = path.getStartVertex();
            }

            edges.addAll(path.getEdgeList());
            weight += path.getWeight();

            end = path.getEndVertex();
        }

        return new GraphPathImpl<V,E>(graph, start, end, edges, weight);
    }

}
