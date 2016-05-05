package cz.agents.alite.trajectorytools.graph.jointspatial.rrtstar;

import java.util.Collection;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.util.WaypointTime;
import cz.agents.deconfliction.solver.central.SingleAgentRRTStarSolver;

public class SingleAgentOptimumBiasedJointOnGraphDomain extends JointOnGraphDomain {

    Logger LOGGER = Logger.getLogger(SingleAgentOptimumBiasedJointOnGraphDomain.class);

    SingleAgentRRTStarSolver[] singleAgentSolvers;
    double temperature = 0;
    double tryGoalRatio;

    public SingleAgentOptimumBiasedJointOnGraphDomain(
            DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs,
            SingleAgentRRTStarSolver[] singleAgentSolvers,
            Box4dRegion bounds, Waypoint[] starts, Waypoint[] targets,
            double separation, double maxEdgeDuration, double speed,
            double temperature, double tryGoalRatio, Random random) {
        super(maneuverGraphs, bounds, starts, targets, separation,
                maxEdgeDuration, speed, random);
        this.singleAgentSolvers = singleAgentSolvers;
        this.temperature = temperature;
        this.tryGoalRatio = tryGoalRatio;
    }

    @Override
    public JointWaypointState sampleState() {

        if (random.nextDouble() < tryGoalRatio) {
            return new JointWaypointState(targets);
        }

        if (allSingleAgentPathsFound()) {
            JointWaypointState sample;

            sample = sampleSingleAgentOptimums();

            // it might be that the there are not single agent samples
            return sample;
        } else {
            return null;
        }

    }

    private JointWaypointState sampleSingleAgentOptimums() {
        Waypoint[] waypoints = new Waypoint[nAgents];
        double t = Math.round(getMaxTimeCoveredInAllTrees() * random.nextDouble());

        for (int i=0; i<nAgents(); i++) {
            if (singleAgentSolvers[i] != null) {
                Waypoint onTrajectoryWp = singleAgentSolvers[i].getOptimalWaypointForTime(t);

//            	double alpha = (nAgents()-1)*0.5;
//            	double alpha = Math.log(nAgents())*0.5;
//              double alpha = Math.pow(nAgents()-1, 0.1)*0.5;
                double alpha = 0.5;

//            	System.out.println(alpha);

                double x = onTrajectoryWp.getX() + (random.nextGaussian() *  alpha);
                double y = onTrajectoryWp.getY() + (random.nextGaussian() *  alpha);

                double z = bounds.getCenter().z;
                SpatialPoint point = new SpatialPoint(x, y, z);
                waypoints[i] = SpatialGraphs.getNearestVertex(maneuverGraphs[i], point);
            }
        }

        return new JointWaypointState(waypoints);
    }

    private Waypoint chooseRandomlyFromCandidates(Collection<WaypointTime> candidates) {
        return candidates.toArray(new WaypointTime[0])[random.nextInt(candidates.size())].getWaypoint();
    }

    private double getMaxTimeCoveredInAllTrees() {
        double minMaxTime = Double.POSITIVE_INFINITY;
        for (int i=0; i<nAgents(); i++) {
            if (singleAgentSolvers[i] != null) {
                if (singleAgentSolvers[i].getMaxTimeInTree() <= minMaxTime) {
                    minMaxTime = singleAgentSolvers[i].getMaxTimeInTree();
                }
            }
        }

        return minMaxTime;
    }

    protected boolean allSingleAgentPathsFound() {
        for (SingleAgentRRTStarSolver solver  : singleAgentSolvers) {
            if (solver != null) {
                if (!solver.foundPathToTarget()) {
                    return false;
                }
            }
        }
        return true;
    }





}
