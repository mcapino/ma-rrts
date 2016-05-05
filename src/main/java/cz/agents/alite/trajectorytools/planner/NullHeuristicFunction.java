package cz.agents.alite.trajectorytools.planner;

final class NullHeuristicFunction<V> implements HeuristicFunction<V> {
    @Override
    public double getHeuristicEstimate(V current, V goal) {
        return 0;
    }
}