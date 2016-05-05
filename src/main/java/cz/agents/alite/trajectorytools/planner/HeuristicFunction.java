package cz.agents.alite.trajectorytools.planner;

public interface HeuristicFunction<V> {
    double getHeuristicEstimate(V current, V goal);
}