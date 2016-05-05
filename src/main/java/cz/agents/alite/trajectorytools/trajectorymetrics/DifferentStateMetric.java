package cz.agents.alite.trajectorytools.trajectorymetrics;

import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class DifferentStateMetric<V extends SpatialPoint, E> implements TrajectoryMetric<V, E> {

    public DifferentStateMetric() {
    }

    @Override
    public double getTrajectoryDistance( PlannedPath<V, E> path, PlannedPath<V, E> otherPath) {
        double penalty = 0;
        if (pathContainsVertex(path.getStartVertex(), otherPath)) {
            penalty += 0.5;
        }

        if (pathContainsVertex(path.getEndVertex(), otherPath)) {
            penalty += 0.5;
        }

        for (E edge : path.getEdgeList()) {
            if (pathContainsVertex(path.getGraph().getEdgeSource(edge), otherPath)) {
                penalty += 0.5;
            }
            if (pathContainsVertex(path.getGraph().getEdgeTarget(edge), otherPath)) {
                penalty += 0.5;
            }
        }

        return 1 - penalty / (path.getEdgeList().size() + 1);
    }

    private boolean pathContainsVertex(V vertex, PlannedPath<V, E> path) {
        for (E edge : path.getEdgeList()) {
            if ( vertex.equals(path.getGraph().getEdgeSource(edge)) || vertex.equals(path.getGraph().getEdgeTarget(edge)) ) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String getName() {
        return "Different States";
    }
}
