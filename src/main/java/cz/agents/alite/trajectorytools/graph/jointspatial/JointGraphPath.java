package cz.agents.alite.trajectorytools.graph.jointspatial;

import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointGraphPath {
    GraphPath<Waypoint, SpatialManeuver>[] paths;
    double cost;
    double duration;

    public JointGraphPath(GraphPath<Waypoint, SpatialManeuver>[] paths) {
        super();
        this.paths = paths;

        determineCost();
        determineDuration();
    }

    public double getCost() {
        return cost;
    }

    private void determineCost() {
        cost = 0.0;
        for (int i=0; i<paths.length; i++) {
            if (paths[i] != null) {
                cost += paths[i].getWeight();
            }
        }
    }

    private void determineDuration() {
        duration = 0.0;
        for (int i=0; i<paths.length; i++) {
            if (paths[i] != null) {
                GraphPath<Waypoint, SpatialManeuver> path = paths[i];
                for (SpatialManeuver maneuver : path.getEdgeList()) {
                    duration += maneuver.getDuration();
                }
                break;
            }
        }
    }

    public JointWaypointState getTargetState(JointWaypointState start) {
        Waypoint[] waypoints = new Waypoint[paths.length];

        for (int i = 0; i < paths.length; i++) {
            if (paths[i] != null) {
                waypoints[i] = paths[i].getEndVertex();
            }
        }

        return new JointWaypointState(waypoints);
    }


    public GraphPath<Waypoint, SpatialManeuver> getPath(int i) {
        return paths[i];
    }

    public double getDuration() {
        return duration;
    }


}
