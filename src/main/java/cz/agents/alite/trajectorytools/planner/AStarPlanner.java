package cz.agents.alite.trajectorytools.planner;

import java.util.List;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.graph.GraphDelegator;


/**
 * An implementation of <a href="http://en.wikipedia.org/wiki/A*_search_algorithm">A* search algorithm</a>.
 */
public final class AStarPlanner<V, E> implements PathPlanner<V, E>
{
    private HeuristicFunction<V> functionH = new NullHeuristicFunction<V>();
    private GoalPenaltyFunction<V> functionG = new NullGoalPenaltyFunction<V>();

    @Override
    public PlannedPath<V, E> planPath(final Graph<V, E> graph, final V startVertex,
            final V endVertex) {
        return planPath(graph, startVertex, endVertex, functionG, functionH);
    }

    @Override
    public PlannedPath<V, E> planPath(final Graph<V, E> graph, final V startVertex,
            final V endVertex, final GoalPenaltyFunction<V> functionG, final HeuristicFunction<V> functionH) {

        GraphWithPenaltyFunction<V, E> penaltyGraph;

        if (graph instanceof DirectedGraph<?,?>) {
            penaltyGraph = new DirectedGraphWithPenaltyFunction<V, E>(graph, functionG);
        } 	else {
            penaltyGraph = new GraphWithPenaltyFunction<V, E>(graph, functionG);
        }

        try {
            AStarShortestPath<V, E> aStar = new AStarShortestPath<V, E>(penaltyGraph, startVertex, endVertex, new AStarShortestPath.Heuristic<V>() {
                @Override
                public double getHeuristicEstimate(V current, V goal) {
                    return functionH.getHeuristicEstimate(current, goal);
                }
            });

            List<E> pathEdgeList = aStar.getPathEdgeList();
            if (pathEdgeList != null) {
                return new PlannedPathImpl<V, E>(graph, pathEdgeList);
            } else {
                return null;
            }
        } catch (Exception e) {
            return null;
        }
    }

    @Override
    public void setGoalPenaltyFunction(GoalPenaltyFunction<V> functionG) {
        this.functionG = functionG;
    }

    @Override
    public void setHeuristicFunction(HeuristicFunction<V> functionH) {
        this.functionH = functionH;
    }

    static class GraphWithPenaltyFunction<V, E> extends GraphDelegator<V, E> {
        private static final long serialVersionUID = -3985698807336517743L;
        private final GoalPenaltyFunction<V> functionG;

        public GraphWithPenaltyFunction(Graph<V, E> g, GoalPenaltyFunction<V> functionG) {
            super(g);
            this.functionG = functionG;
        }

        @Override
        public double getEdgeWeight(E e) {
            // return super.getEdgeWeight(e) + functionG.getGoalPenalty((V) e.getSource()) + functionG.getGoalPenalty((V) e.getTarget());
            return super.getEdgeWeight(e) + functionG.getGoalPenalty(super.getEdgeTarget(e));
        }

    }

    static class DirectedGraphWithPenaltyFunction<V, E> extends GraphWithPenaltyFunction<V, E> implements DirectedGraph<V,E> {
        private static final long serialVersionUID = 8654284725272084227L;

        public DirectedGraphWithPenaltyFunction(Graph<V, E> g, GoalPenaltyFunction<V> functionG) {
            super(g, functionG);
        }
    }

    @Override
    public HeuristicFunction<V> getHeuristicFunction() {
        return this.functionH;
    }
}
