package cz.agents.alite.trajectorytools.planner;

import org.jgrapht.Graph;

public interface PathPlanner<V, E> {

    /**
     * @param graph the graph to be searched
     * @param startVertex the vertex at which the path should start
     * @param endVertex the vertex at which the path should end
     * 
     * @return planned path or null if no path exists
     */
    public PlannedPath<V, E> planPath(final Graph<V, E> graph, final V startVertex,
            final V endVertex);

    public void setGoalPenaltyFunction(GoalPenaltyFunction<V> functionG);
    public void setHeuristicFunction(HeuristicFunction<V> functionH);

    public HeuristicFunction<V> getHeuristicFunction();

    /**
     * State-less (and thus thread-safe) version of the planPath method. 
     * @param graph
     * @param startVertex
     * @param endVertex
     * @param functionG
     * @param functionH
     * @return
     */
    public PlannedPath<V, E> planPath(final Graph<V, E> graph, final V startVertex,
            final V endVertex, final GoalPenaltyFunction<V> functionG, final HeuristicFunction<V> functionH);

}