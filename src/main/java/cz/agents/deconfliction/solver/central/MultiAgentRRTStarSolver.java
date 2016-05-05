package cz.agents.deconfliction.solver.central;

import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.jointspatial.rrtstar.BiasedJointOnGraphDomain;
import cz.agents.alite.trajectorytools.graph.spatial.FreeWhenOnTargetGraph;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.JointGraphPathToTrajectoriesConverter;

public class MultiAgentRRTStarSolver extends CentralCooperativePathfindingSolver {

    Logger LOGGER = Logger.getLogger(MultiAgentRRTStarSolver.class);

    Domain<JointWaypointState, JointGraphPath> domain;

    private double bestCost = Double.POSITIVE_INFINITY;
    private EvaluatedTrajectory[] bestTrajectories = null;
    private RRTStarPlanner<JointWaypointState, JointGraphPath> rrtstar = null;

    private DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs;

    public int iterations = 0;

    @SuppressWarnings("unchecked")
    public MultiAgentRRTStarSolver(
            DirectedGraph<Waypoint, SpatialManeuver> maneuverGraph, Box4dRegion bounds,
            Waypoint[] starts, Waypoint[] targets, double separation, double maxTime, double vmax, double gamma, double maxEdgeLength, double tryGoalRatio) {
        super(maneuverGraph, targets, separation, maxTime, vmax);

        this.maneuverGraphs = new DirectedGraph[targets.length];

        for (int i=0; i<targets.length; i++) {
            if (targets[i] != null) {
                this.maneuverGraphs[i] = new FreeWhenOnTargetGraph<Waypoint, SpatialManeuver>(maneuverGraph, targets[i]);
            }
        }

        domain = new BiasedJointOnGraphDomain(maneuverGraphs, bounds, starts, targets, separation, maxEdgeLength, vmax, tryGoalRatio, new Random(1));
        rrtstar = new RRTStarPlanner<JointWaypointState, JointGraphPath>(domain, new JointWaypointState(starts), gamma, maxEdgeLength * nAgents);
    }

    public EvaluatedTrajectory[] solve(double runtimeLimitMs) {
        step((long) runtimeLimitMs * (long)1e6);
        return bestTrajectories;
    }

    public void step(long runtimeLimitNs) {
        long interruptAtNs = System.nanoTime() + runtimeLimitNs;

        while(System.nanoTime() < interruptAtNs) {
            rrtstar.iterate();
            //System.out.println(Counters.statesExpanded);
            iterations ++;
            Counters.iterations++;

            //try {   Thread.sleep(500);    } catch (InterruptedException e) {}

            if (rrtstar.getBestVertex() != null && rrtstar.getBestVertex().getCostFromRoot() < bestCost) {
                bestCost = rrtstar.getBestVertex().getCostFromRoot();
                LOGGER.trace("Iteration: " + iterations + " Best path cost: " + bestCost);
                GraphPath<JointWaypointState, JointGraphPath> path = rrtstar.getBestPath();
                bestTrajectories = JointGraphPathToTrajectoriesConverter.convert(path, maneuverGraphs, maxTime);
                notifyNewSolution(bestTrajectories, false);
            }

        }
    }

    public RRTStarPlanner<JointWaypointState, JointGraphPath> getRRTStarPlanner() {
        return rrtstar;
    }

}
