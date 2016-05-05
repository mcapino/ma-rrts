package cz.agents.alite.trajectorytools.graph.spatial;

import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.jgrapht.EdgeFactory;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
import org.jgrapht.alg.specifics.Specifics;
import org.jgrapht.alg.specifics.SpecificsFactory;
import org.jgrapht.graph.DirectedWeightedMultigraph;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class SpatialGraphs {
    public static <V extends SpatialPoint, E> V getNearestVertex(Graph<V, E> graph, SpatialPoint pos) {
        V nearestVertex = null;
        double nearestDistance = Double.POSITIVE_INFINITY;
        for (V currentVertex : graph.vertexSet()) {
            double distance = currentVertex.distance(pos);
            if (distance < nearestDistance || nearestVertex == null) {
                nearestVertex = currentVertex;
                nearestDistance = distance;
            }
        }

        return nearestVertex;
    }

    @SuppressWarnings("unchecked")
    public static <V extends SpatialPoint, E> V getRandomVertex(Graph<V, E> graph, Random random) {
        int n = random.nextInt(graph.vertexSet().size());
        return (V) graph.vertexSet().toArray()[n];
    }

    /**
     * Runs a greedy search guided by a given heuristic function through the
     * graph towards a specified goal. The traversal is terminated if the
     * cost exceeds maxCost.
     *
     * The traversal may be terminated for three reasons: 1) the target was reached
     * 2) cost of path exceeded maxCost 3) heuristic returns Double.Positive infinity
     * for all neighbors.
     */
    public static <V extends SpatialPoint, E> GraphPath<V,E> greedySearch(Graph<V, E> graph, V start, V target, HeuristicFunction<V> heuristic, double maxCost) {
        Specifics<V, E> specifics = SpecificsFactory.create(graph);
        List<E> edges = new LinkedList<E>();

        V current = start;
        double cost = 0.0;

        while (!current.equals(target) && cost < maxCost) {
            Set<? extends E> outgoingEdges = specifics.edgesOf(current);

            E bestEdge = null;
            V bestNeighbor = null;
            double bestHValue = Double.POSITIVE_INFINITY;

            // find an edge that leads to a vertex having the highest heuristic value

            for (E edge : outgoingEdges) {
                V neighbor = Graphs.getOppositeVertex(graph, edge, current);
                double hValue = heuristic.getHeuristicEstimate(neighbor, target);

                if (hValue < bestHValue) {
                    bestHValue = hValue;
                    bestEdge = edge;
                    bestNeighbor = neighbor;
                }
            }

            if (bestEdge != null) {
                edges.add(bestEdge);
                current = bestNeighbor;
                cost += graph.getEdgeWeight(bestEdge);
            } else {
                break;
            }

        }

        return new GraphPathImpl<V,E>(graph, start, current, edges, cost);
    }

    public static <V, E> Graph<V, E>  clone(Graph<V, E> other) {
        SimpleGraphWithObstacles<V, E> graph = new SimpleGraphWithObstacles<V,E>(other.getEdgeFactory());

        for (V vertex : other.vertexSet()) {
            graph.addVertex(vertex);
        }

        for (E edge : other.edgeSet()) {
            graph.addEdge(other.getEdgeSource(edge), other.getEdgeTarget(edge), edge);
        }

        if (other instanceof GraphWithObstacles) {
            graph.addObstacles(((GraphWithObstacles<V, ?>) other).getObstacles() );
        }
        return graph;
    }

    public static <V extends SpatialPoint, E> void cutOutObstacles(Graph<V, E> graph, Collection<SpaceRegion> obstacles) {
        Set<V> vertices = new HashSet<V>(graph.vertexSet());
        for (V vertex : vertices) {
            if (isInObstacle(vertex, obstacles)) {
                graph.removeVertex(vertex);
            }
        }
    };

    public static boolean isInObstacle(SpatialPoint point, Collection<SpaceRegion> obstacles) {
        for (SpaceRegion obstacle : obstacles) {
            if (obstacle.isInside(point)) {
                return true;
            }
        }
        return false;
    }
}


class SimpleGraphWithObstacles<V, E> extends DirectedWeightedMultigraph<V, E> implements GraphWithObstacles<V, E> {
    private static final long serialVersionUID = 1L;

    Set<V> obstacles = new HashSet<V>();

    public SimpleGraphWithObstacles(EdgeFactory<V, E> edgeFactory) {
        super(edgeFactory);
    }

    @Override
    public Set<V> getObstacles() {
        return obstacles;
    }

    @Override
    public void addObstacle(V obstacle) {
        obstacles.add(obstacle);
    }

    public void addObstacles(Collection<V> obstacle) {
        obstacles.addAll(obstacle);
    }

    @Override
    public void refresh() {
        throw new UnsupportedOperationException("Not implemented");
    }

}
