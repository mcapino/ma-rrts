package cz.agents.alite.trajectorytools.planner;

public interface DistanceFunction<V> {
    double getDistance(V current, V goal);
}