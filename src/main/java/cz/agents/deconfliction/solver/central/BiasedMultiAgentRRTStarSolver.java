package cz.agents.deconfliction.solver.central;

import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.jointspatial.rrtstar.SingleAgentOptimumBiasedJointOnGraphDomain;
import cz.agents.alite.trajectorytools.graph.spatial.FreeWhenOnTargetGraph;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.JointGraphPathToTrajectoriesConverter;

public class BiasedMultiAgentRRTStarSolver extends CentralCooperativePathfindingSolver {

    Logger LOGGER = Logger.getLogger(BiasedMultiAgentRRTStarSolver.class);
    private Random random = new Random(1);
    Domain<JointWaypointState, JointGraphPath> domain;

    private double bestCost = Double.POSITIVE_INFINITY;
    private EvaluatedTrajectory[] bestTrajectories = null;
    private RRTStarPlanner<JointWaypointState, JointGraphPath> rrtstar = null;

    private DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs;
    private SingleAgentRRTStarSolver[] singleAgentSolvers;

    
    public int iterations = 0;
    

    @SuppressWarnings("unchecked")
    public BiasedMultiAgentRRTStarSolver(
            DirectedGraph<Waypoint, SpatialManeuver> maneuverGraph, Box4dRegion bounds,
            Waypoint[] starts, Waypoint[] targets, double separation, double maxTime, double vmax, double gamma, double maxEdgeLength, double tryGoalRatio) {
        super(maneuverGraph, targets, separation, maxTime, vmax);

        this.maneuverGraphs = new DirectedGraph[targets.length];

        for (int i=0; i<targets.length; i++) {
            if (targets[i] != null) {
                this.maneuverGraphs[i]
                        = new FreeWhenOnTargetGraph<Waypoint, SpatialManeuver>(maneuverGraph, targets[i]);
            }
        }

        // Initialize single agent solvers
        singleAgentSolvers = new SingleAgentRRTStarSolver[nAgents];
        for (int i=0; i<nAgents; i++) {
            if (targets[i] != null) {

                singleAgentSolvers[i] = new SingleAgentRRTStarSolver(
                        maneuverGraphs[i], bounds.get3dBounds(), starts[i],
                        targets[i], maxTime, gamma / nAgents, maxEdgeLength,
                        tryGoalRatio, random);

            }
        }
        
        double temperature = 20;
        domain = new SingleAgentOptimumBiasedJointOnGraphDomain(maneuverGraphs,
                singleAgentSolvers, bounds, starts, targets, separation,
                maxEdgeLength, vmax, temperature, tryGoalRatio, random);
        rrtstar = new RRTStarPlanner<JointWaypointState, JointGraphPath>(domain, new JointWaypointState(starts), gamma, maxEdgeLength * nAgents);
    }

    public EvaluatedTrajectory[] solve(long runtimeLimitMs) {
        step((long) runtimeLimitMs * (long)1e6);
        return bestTrajectories;
    }

    public void step(long runtimeLimitNs) {
        long interruptAtNs = System.nanoTime() + runtimeLimitNs;

        while(System.nanoTime() < interruptAtNs) {
            iterate();
        }
    }

    private void iterate() {
        iterateSingleAgentPlanners(1);
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

    private void iterateSingleAgentPlanners(int iterataions) {
        for (int j=0; j<iterataions; j++) {
            for (int i=0; i<singleAgentSolvers.length; i++) {
                singleAgentSolvers[i].iterate();
            }
        }
    }

    public RRTStarPlanner<JointWaypointState, JointGraphPath> getRRTStarPlanner() {
        return rrtstar;
    }

    public SingleAgentRRTStarSolver[] getSingleAgentSolvers() {
        return singleAgentSolvers;
    }

}
