package cz.agents.alite.trajectorytools.graph.jointspatiotemporal;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;

public class JointSpatioTemporalManeuver {

    SpatioTemporalManeuver[] maneuvers;

    public JointSpatioTemporalManeuver(SpatioTemporalManeuver[] maneuvers) {
        this.maneuvers = maneuvers;
    }

    public SpatioTemporalManeuver get(int i) {
        return maneuvers[i];
    }

    public int nAgents() {
        return maneuvers.length;
    }

}
