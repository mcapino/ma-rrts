package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
import org.jgrapht.alg.specifics.Specifics;
import org.jgrapht.alg.specifics.SpecificsFactory;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.DistanceFunction;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.deconfliction.solver.central.Counters;

public class ManeuverGraphDomain<S extends SpatialPoint, E extends SpatialManeuver> implements Domain<S, GraphPath<S,E>> {


    protected Graph<S,E> graph;
    protected S target;

    protected final DistanceFunction<S> distance;

    protected double maxExtensionCost;
    protected Random random;

    protected HeuristicFunction<S> h;

    public ManeuverGraphDomain(Graph<S, E> graph, S target,
            final DistanceFunction<S> distance, double maxExtensionCost, Random random) {
        super();
        this.graph = graph;
        this.target = target;
        this.distance = distance;
        this.maxExtensionCost = maxExtensionCost;
        this.random = random;

        this.h = new HeuristicFunction<S>() {

            @Override
            public double getHeuristicEstimate(S current, S goal) {
                return ManeuverGraphDomain.this.distance.getDistance(current, goal);
            }
        };
    }

    @Override
    public S sampleState() {
        return SpatialGraphs.getRandomVertex(graph, random);
    }

    @Override
    public Extension<S, GraphPath<S,E>> extendTo(S from, S to) {
        GraphPath<S, E> path = greedyOptimalSearch(graph, from, to, h, maxExtensionCost);
        boolean exact = path.getEndVertex().equals(to);
        return new Extension<S, GraphPath<S,E>>(from, path.getEndVertex(), path, path.getWeight(), exact);
    }

    @Override
    public ExtensionEstimate estimateExtension(S from, S to) {
        double cost = distance.getDistance(from, to);
        return new ExtensionEstimate(cost, cost <= maxExtensionCost);
    }

    @Override
    public double estimateCostToGo(S s) {
        return h.getHeuristicEstimate(s, target);
    }

    @Override
    public double distance(S s1, S s2) {
        return distance.getDistance(s1, s2);
    }

    @Override
    public double nDimensions() {
        return 3;
    }

    @Override
    public boolean isInTargetRegion(S s) {
        return target.equals(s);
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
            Counters.statesExpanded++;
            Set<? extends E> outgoingEdges = specifics.edgesOf(current);

            E bestEdge = null;
            V bestNeighbor = null;
            double bestHValue = Double.POSITIVE_INFINITY;

            // find an edge that leads to a vertex having the highest heuristic value

            for (E edge : outgoingEdges) {
                V neighbor = Graphs.getOppositeVertex(graph, edge, current);
                double hValue = heuristic.getHeuristicEstimate(neighbor, target);

                // ignore wait moves
                if (neighbor.equals(current)) continue;

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

    /**
     * Runs a greedy search guided by a given heuristic function through the
     * graph towards a specified goal. The traversal is terminated if the
     * cost exceeds maxCost.
     *
     * The traversal may be terminated for four reasons: 1) the target was reached
     * 2) the path becomes suboptimal
     * 3) cost of path exceeded maxCost
     * 4) heuristic returns Double.Positive infinity
     * for all neighbors.
     */
    public static <V extends SpatialPoint, E> GraphPath<V,E> greedyOptimalSearch(Graph<V, E> graph, V start, V target, HeuristicFunction<V> heuristic, double maxCost) {
        Specifics<V, E> specifics = SpecificsFactory.create(graph);
        List<E> edges = new LinkedList<E>();

        V current = start;
        double pathCost = 0.0;

        while (!current.equals(target) && pathCost < maxCost) {
            Counters.statesExpanded++;
            Set<? extends E> outgoingEdges = specifics.edgesOf(current);

            E bestEdge = null;
            V bestNeighbor = null;
            double bestHValue = Double.POSITIVE_INFINITY;

            // find an edge that leads to a vertex having the highest heuristic value

            for (E edge : outgoingEdges) {
                V neighbor = Graphs.getOppositeVertex(graph, edge, current);
                double hValue = heuristic.getHeuristicEstimate(neighbor, target);
                double startToNeighborLowerBoundCost = heuristic.getHeuristicEstimate(start, neighbor);
                double edgeCost = graph.getEdgeWeight(edge);

                // ignore wait moves
                if (neighbor.equals(current)) continue;

                if (hValue < bestHValue && startToNeighborLowerBoundCost >= (pathCost + edgeCost - 0.001)) {
                    bestHValue = hValue;
                    bestEdge = edge;
                    bestNeighbor = neighbor;
                }
            }

            if (bestEdge != null) {
                edges.add(bestEdge);
                current = bestNeighbor;
                pathCost += graph.getEdgeWeight(bestEdge);
            } else {
                break;
            }

        }

        return new GraphPathImpl<V,E>(graph, start, current, edges, pathCost);
    }

}
