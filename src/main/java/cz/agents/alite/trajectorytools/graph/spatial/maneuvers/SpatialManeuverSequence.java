package cz.agents.alite.trajectorytools.graph.spatial.maneuvers;

import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class SpatialManeuverSequence<V extends SpatialPoint> extends SpatialManeuver {

    GraphPath<V, SpatialManeuver> path;
    double duration = 0.0;
    double distance = 0.0;

    public SpatialManeuverSequence() {
        for (SpatialManeuver edge : path.getEdgeList()) {
            distance += edge.getDistance();
            duration += edge.getDuration();
        }
    }

    @Override
    public Trajectory getTrajectory(double startTime) {
        return new SpatialManeuverTrajectory<V, SpatialManeuver>(startTime, path, duration);
    }

    @Override
    public double getDistance() {
        return distance;
    }

    @Override
    public double getDuration() {
        return duration;
    }

}
