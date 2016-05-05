package cz.agents.deconfliction.solver.central;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.GraphPaths;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.rrtstar.BiasedManeuverGraphDomain;
import cz.agents.alite.trajectorytools.planner.DistanceFunction;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarListener;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner.Condition;
import cz.agents.alite.trajectorytools.planner.rrtstar.Vertex;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.util.WaypointTime;

public class SingleAgentRRTStarSolver {

    double maxTimeInRRTStarTree = 0.0;

    public interface Listener {
        public abstract void notifyNewSolution(EvaluatedTrajectory trajectories);
    }


    Logger LOGGER = Logger.getLogger(SingleAgentRRTStarSolver.class);

    Domain<Waypoint, GraphPath<Waypoint, SpatialManeuver>> domain;

    private double bestCost = Double.POSITIVE_INFINITY;
    private EvaluatedTrajectory bestTrajectory = null;
    private GraphPath<Waypoint, SpatialManeuver> bestPath;
    private RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstar = null;
    private DirectedGraph<Waypoint, SpatialManeuver> maneuverGraph;
    private double maxTime;

    public int iterations = 0;
    private Waypoint target;

    List<Listener> listeners = new LinkedList<Listener>();



    public SingleAgentRRTStarSolver(
            DirectedGraph<Waypoint, SpatialManeuver> maneuverGraph, BoxRegion bounds,
            Waypoint start, Waypoint target, double maxTime, double gamma, double maxEdgeLength, double tryGoalRatio, Random random) {
        super();

        this.maneuverGraph = maneuverGraph;
        this.maxTime = maxTime;
        this.target = target;

        DistanceFunction<Waypoint> distanceFunction = new DistanceFunction<Waypoint>() {

            @Override
            public double getDistance(Waypoint current, Waypoint goal) {
                return current.distanceL1(goal);
            }
        };

        domain = new BiasedManeuverGraphDomain<Waypoint, SpatialManeuver>(maneuverGraph, target, distanceFunction, maxEdgeLength, tryGoalRatio, random);
        rrtstar = new RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>>(domain, start, gamma, maxEdgeLength);
        rrtstar.registerListener(new RRTStarListener<Waypoint, GraphPath<Waypoint, SpatialManeuver>> () {

            @Override
            public void notifyNewVertexInTree(
                    Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> v) {

                if (v.getCostFromRoot() > maxTimeInRRTStarTree) {
                    maxTimeInRRTStarTree = v.getCostFromRoot();
                }
            }
        });
    }

    public EvaluatedTrajectory solve(double runtimeLimitMs) {
        step((long) runtimeLimitMs * (long)1e6);
        return bestTrajectory;
    }

    public void step(long runtimeLimitNs) {
        long interruptAtNs = System.nanoTime() + runtimeLimitNs;

        while(System.nanoTime() < interruptAtNs) {

            iterate();
        }
    }

    public void iterate() {
        rrtstar.iterate();
        //System.out.println(Counters.statesExpanded);
        iterations ++;
        Counters.iterations++;

        if (rrtstar.getBestVertex() != null && rrtstar.getBestVertex().getCostFromRoot() < bestCost) {

            bestCost = rrtstar.getBestVertex().getCostFromRoot();

            //LOGGER.trace("Iteration: " + iterations + " Best path cost: " + bestCost);
            GraphPath<Waypoint, GraphPath<Waypoint, SpatialManeuver>> path = rrtstar.getBestPath();

            bestPath = GraphPaths.concatenate(maneuverGraph, path.getEdgeList());
            bestTrajectory = new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(0.0, bestPath, maxTime);

            notifyNewSolution(bestTrajectory);
        }
    }





    protected void notifyNewSolution(EvaluatedTrajectory trajectory) {
        for (Listener listener : listeners) {
            listener.notifyNewSolution(trajectory);
        }
    }

    public void registerListener(Listener listener) {
        listeners.add(listener);
    }

    public RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> getRRTStarPlanner() {
        return rrtstar;
    }

    public Collection<WaypointTime> getWaypointsInTimeInterval(final double timeMin, final double timeMax) {
        Collection<WaypointTime> waypoints = new LinkedList<WaypointTime>();
        Collection<Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>>> vertices = rrtstar.conditionSearch(new Condition<Waypoint, GraphPath<Waypoint,SpatialManeuver>>() {

            @Override
            public boolean satisfiesCondition(
                    Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> examinedVertex) {
                double vertexTime = examinedVertex.getCostFromRoot();
                return vertexTime >= (timeMin-0.01) && vertexTime <= (timeMax+0.01);
            }

        });

        for (Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> vertex : vertices) {
            waypoints.add(new WaypointTime(vertex.getState(), vertex.getCostFromRoot()));
        }

        // it is okay to stay on the target
        if (foundPathToTarget() && getBestArrivalTimeToTarget() < timeMin) {
            waypoints.add(new WaypointTime(target, timeMin));
        }

        return waypoints;
    }

    public Collection<Waypoint> getWaypointsReachableInTime(final double time) {
        Collection<Waypoint> waypoints = new LinkedList<Waypoint>();
        Collection<Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>>> vertices = rrtstar
                .conditionSearch(new Condition<Waypoint, GraphPath<Waypoint, SpatialManeuver>>() {

            @Override
            public boolean satisfiesCondition(
                    Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> examinedVertex) {
                double vertexTime = examinedVertex.getCostFromRoot(); // beware a little workaround
                return vertexTime <= (time + 0.01);
            }

        });

        for (Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> vertex : vertices) {
            waypoints.add(vertex.getState());
        }

        return waypoints;
    }

    public double getMaxTimeInTree() {
        return maxTimeInRRTStarTree;
    }

    public double getBestArrivalTimeToTarget(){
        return bestCost;
    }

    public boolean foundPathToTarget() {
        return bestTrajectory != null;
    }

    public Waypoint getOptimalWaypointForTime(double t) {
        if (bestPath != null) {
            List<SpatialManeuver> edges = bestPath.getEdgeList();
            double edgeTargetTime = 0;
            for (SpatialManeuver edge : edges) {
                edgeTargetTime += edge.getDuration();
                if (MathUtil.equals(edgeTargetTime, t, 0.1)) {
                    return bestPath.getGraph().getEdgeTarget(edge);
                }
            }

            // the time provided is after we arrived to the target destination
            return bestPath.getEndVertex();
        }
        return null;
    }

}
