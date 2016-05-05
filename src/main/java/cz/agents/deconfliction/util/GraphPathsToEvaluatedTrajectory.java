package cz.agents.deconfliction.util;

import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectoryWrapper;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class GraphPathsToEvaluatedTrajectory {

    public static EvaluatedTrajectory[] convert(
            GraphPath<Waypoint, SpatialManeuver>[] paths, double[] startTimes,
            double maxTime) {
        if (paths == null) {
            return null;
        }

        EvaluatedTrajectory[] trajectories = new EvaluatedTrajectory[paths.length];
        for (int i=0; i < paths.length; i++) {
            if (paths[i] != null) {
                trajectories[i] = new EvaluatedTrajectoryWrapper(
                                    new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(startTimes[i], paths[i], maxTime - startTimes[i])
                            ,paths[i].getWeight());
            }
        }

        return trajectories;
    }
}
