package cz.agents.alite.trajectorytools.graph.jointspatial;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;

public class JointSpatialManeuver {

    SpatialManeuver[] maneuvers;

    public JointSpatialManeuver(SpatialManeuver[] maneuvers) {
        this.maneuvers = maneuvers;
    }

    public SpatialManeuver get(int i) {
        return maneuvers[i];
    }

    public int nAgents() {
        return maneuvers.length;
    }

}
