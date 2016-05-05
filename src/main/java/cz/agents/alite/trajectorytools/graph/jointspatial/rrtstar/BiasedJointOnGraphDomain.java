package cz.agents.alite.trajectorytools.graph.jointspatial.rrtstar;

import java.util.Random;

import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class BiasedJointOnGraphDomain extends JointOnGraphDomain {

    double tryGoalRatio;

    public BiasedJointOnGraphDomain(
            DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs,
            Box4dRegion bounds, Waypoint[] starts, Waypoint[] targets, double separation,
            double maxEdgeDuration, double speed, double tryGoalRatio, Random random) {
        super(maneuverGraphs, bounds, starts, targets, separation, maxEdgeDuration, speed, random);

        this.tryGoalRatio = tryGoalRatio;
    }

    @Override
    public JointWaypointState sampleState() {

        if (random.nextDouble() < tryGoalRatio) {
            return new JointWaypointState(targets);
        }
        else {
            return super.sampleState();
        }
    }

}
