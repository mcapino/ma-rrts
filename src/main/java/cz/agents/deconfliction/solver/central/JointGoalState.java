package cz.agents.deconfliction.solver.central;

import cz.agents.alite.trajectorytools.util.Waypoint;


public class JointGoalState extends JointODState {

    public JointGoalState(Waypoint[] waypoints) {
    	super(ODState.fromWaypoints(waypoints), null, 0.0, 0.0, 0);
    }
}
