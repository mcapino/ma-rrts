package cz.agents.alite.trajectorytools.graph.jointspatial.rrtstar;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.Graphs;
import org.jgrapht.alg.specifics.Specifics;
import org.jgrapht.alg.specifics.SpecificsFactory;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.solver.central.Counters;

public class JointOnGraphDomain implements Domain<JointWaypointState, JointGraphPath> {


    DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs;
    protected Box4dRegion bounds;
    int nAgents;
    protected Waypoint[] starts;
    protected double separation;
    protected Random random;
    protected double samplingInterval;
    protected double speed = 1.0;
    protected Waypoint[] targets;
    protected double maxEdgeDuration;



    public JointOnGraphDomain(
            DirectedGraph<Waypoint, SpatialManeuver>[] maneuverGraphs,
            Box4dRegion bounds, Waypoint[] starts, Waypoint[] targets, double separation,
             double maxEdgeDuration, double speed, Random random) {
        super();

        this.maneuverGraphs = maneuverGraphs;
        this.bounds = bounds;
        this.nAgents = targets.length;
        this.separation = separation;
        this.samplingInterval = separation / 4.0;
        this.random = random;
        this.speed = speed;
        this.starts = starts;
        this.targets = targets;
        this.maxEdgeDuration = maxEdgeDuration;
    }

    @Override
    public JointWaypointState sampleState() {

        Waypoint[] waypoints = new Waypoint[nAgents];

        // pick spatial point for each agent
        for (int i = 0; i < nAgents(); i++) {

            double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
            double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
            double z = bounds.getCenter().z;
            SpatialPoint point = new SpatialPoint(x, y, z);
            waypoints[i] = SpatialGraphs.getNearestVertex(maneuverGraphs[i], point);
        }

        return new JointWaypointState(waypoints);
    }

    public int nAgents() {
        return nAgents;
    }

    @Override
    public Extension<JointWaypointState, JointGraphPath> extendTo(JointWaypointState from, JointWaypointState to) {
        GraphPath<Waypoint, SpatialManeuver>[] paths = greedySearch(from, to, maxEdgeDuration);
        if (paths != null) {
            JointGraphPath jointGraphPath = new JointGraphPath(paths);
            JointWaypointState target = jointGraphPath.getTargetState(from);

            boolean exact = to.equals(target);

            return new Extension<JointWaypointState, JointGraphPath>(from, target, jointGraphPath, jointGraphPath.getCost(), exact);
        } else {
            return null;
        }
    }


    @Override
    public ExtensionEstimate estimateExtension(JointWaypointState from, JointWaypointState to) {
            Extension<JointWaypointState, JointGraphPath> extension = extendTo(from, to);
            if (extension != null) {
                return new ExtensionEstimate(extension.cost, extension.exact);
            } else {
                return null;
            }
    }

    @Override
    public double estimateCostToGo(JointWaypointState s) {
        double minCost = 0.0;

        for (int i=0; i<targets.length; i++) {
            if (targets[i] != null) {
                minCost += JointWaypointState.singleAgentMinCostDistance(s.getPosition(i), targets[i], speed);
            }
        }

        return minCost;
    }

    @Override
    public double distance(JointWaypointState s1, JointWaypointState s2) {
        return s1.minCostDistance(s2, speed);
    }

    @Override
    public double nDimensions() {
        return 2*nAgents;
    }

    @Override
    public boolean isInTargetRegion(JointWaypointState s) {
        return Arrays.equals(s.getPositions(), targets);
    }

    private double singleAgentDistance(Waypoint start, Waypoint end) {
        return start.distanceL1(end) / speed;
    }

    private GraphPath<Waypoint,SpatialManeuver>[] greedySearch(JointWaypointState start, JointWaypointState target, double maxPathDuration) {

        long ns1 = System.nanoTime();
        ++Counters.greedySearchNum;

        int nAgents = start.nAgents();
        List<SpatialManeuver>[] edges = new LinkedList[nAgents];

        for (int i=0; i<start.nAgents(); i++) {
            if (start.getPosition(i) != null) {
                edges[i] = new LinkedList<SpatialManeuver>();
            }
        }

        Waypoint[] currentPositions = Arrays.copyOf(start.getPositions(), start.nAgents());
        double[] costs = new double[nAgents];
        double jointCost = 0.0;
        double jointDistance = 0.0;

        double pathDuration = 0;
        int stepCounter = 0;

        /*
         *  Greedily traverse until a) in the target space-time state or b) maxPathLength exceeded
         *
         *  Rules:
         *  > Wait move allowed only at target space-time state
         *
         */
        while (!(Arrays.equals(currentPositions, target.getPositions()))
                && pathDuration <= maxPathDuration) {

            Waypoint[] bestNeighbors = new Waypoint[target.nAgents()];
            SpatialManeuver[] bestEdges = new SpatialManeuver[target.nAgents()];
            double[] bestEdgesCosts = new double[target.nAgents()];

            // For each agent find best move
            for (int i = 0; i < nAgents; i++) {

                long n1 = System.nanoTime();
                Specifics<Waypoint, SpatialManeuver> specifics = SpecificsFactory.create(maneuverGraphs[i]);
                long n2 = System.nanoTime();
                Counters.createSpecificsNanos += n2-n1;


                n1 = System.nanoTime();
                Set<? extends SpatialManeuver> outgoingEdges = specifics.edgesOf(currentPositions[i]);
                n2 = System.nanoTime();
                Counters.edgesOfNanos += n2-n1;

                Counters.statesExpanded++;

                SpatialManeuver bestEdge = null;
                Waypoint bestNeighbor = null;
                double bestHValue = Double.POSITIVE_INFINITY;

                // for each agent find an edge that leads to a vertex having the
                // highest heuristic value
                for (SpatialManeuver edge : outgoingEdges) {
                    n1 = System.nanoTime();
                    Waypoint neighbor = Graphs.getOppositeVertex(maneuverGraphs[i], edge, currentPositions[i]);
                    n2 = System.nanoTime();
                    Counters.oppositeVertexNanos += n2-n1;

                    // Waiting is allowed only at the end of the path
                    if (neighbor.equals(currentPositions[i]) && !neighbor.equals(target.getPosition(i))) {
                        continue;
                    }


                    double hValue
                        = singleAgentDistance(neighbor, target.getPosition(i));

                    // slight randomization of the heuristic is designed break ties randomly in order to
                    // to promote exploration of different equal-cost paths from one node
                    hValue += 0.01 * random.nextDouble();

                    if (hValue < bestHValue) {
                        bestHValue = hValue;
                        bestEdge = edge;
                        bestNeighbor = neighbor;
                    }
                }

                if (bestEdge != null) {
                    bestEdges[i] = bestEdge;
                    bestNeighbors[i] = bestNeighbor;
                    bestEdgesCosts[i] = maneuverGraphs[i].getEdgeWeight(bestEdge);
                    assert (MathUtil.equals(bestEdge.getDuration(), 1.0, 0.001));
                } else {
                    throw new RuntimeException("Greedy search stuck");
                }
            }

            // check consistency
            Set<Trajectory> trajectories = new HashSet<Trajectory>();
            double jointEdgeCost = 0.0;
            double jointEdgeDistance = 0.0;


            for (int i=0; i<bestEdges.length; i++) {
                if (bestEdges[i] != null) {
                    trajectories.add(bestEdges[i].getTrajectory(pathDuration));
                    jointEdgeCost += maneuverGraphs[i].getEdgeWeight(bestEdges[i]);
                    jointEdgeDistance += bestEdges[i].getDistance();
                }
            }

            //double lowerBoundCostFromStart = start.minCostDistance(bestNeighbors, speed);
            double lowerBoundDistanceFromStart = start.lowerBoundOnSumDistanceTo(bestNeighbors);

            long n1 = System.nanoTime();
            ++Counters.collisionCheckApproxNum;
            boolean conflicting = SeparationDetector.hasConflict(trajectories, separation, samplingInterval, 1.0);
//            boolean conflicting = SeparationDetector.hasConflictApproximation(trajectories, separation, 1.0);
//            if(conflicting){
//            	conflicting = SeparationDetector.hasConflict(trajectories, separation, samplingInterval);
//            	++Counters.collisionCheckNum;
//            }
            Counters.collisionCheckNanos += System.nanoTime()-n1;

            //boolean optimal = (jointCost + jointEdgeCost - 0.001) <= lowerBoundCostFromStart;
            //boolean distanceOptimal = (jointDistance + jointEdgeDistance - 0.001) <= lowerBoundDistanceFromStart;

            if (!conflicting /*&& distanceOptimal*/) {
                currentPositions = bestNeighbors;
                for (int i=0; i<bestEdges.length; i++) {
                    if (bestEdges[i] != null) {
                        edges[i].add(bestEdges[i]);
                    }
                }

                for (int i=0; i<bestEdgesCosts.length; i++) {
                    if (bestEdges[i] != null) {
                        costs[i] += bestEdgesCosts[i];
                    }
                }

                pathDuration += 1.0;
                stepCounter++;
                jointCost += jointEdgeCost;
                jointDistance += jointEdgeDistance;
            } else {
                break;
            }
        }

        if (stepCounter > 0) {

            @SuppressWarnings("unchecked")
            GraphPath<Waypoint, SpatialManeuver>[] graphPaths = new GraphPath[nAgents];
            for (int i=0; i<nAgents; i++){
                if (start.getPosition(i) != null) {
                    graphPaths[i] = new GraphPathImpl<Waypoint, SpatialManeuver>(maneuverGraphs[i], start.getPosition(i), currentPositions[i], edges[i], costs[i]);
                }
            }

            Counters.greedySearchNanos += System.nanoTime()-ns1;
            return graphPaths;

        } else {

            Counters.greedySearchNanos += System.nanoTime()-ns1;
            return null;
        }




    }


    private static boolean separated(Waypoint[] waypoints, double separation) {
        for (int i=0; i<waypoints.length; i++) {
            for (int j=i+1; j<waypoints.length; j++) {
                if (waypoints[i].distance(waypoints[j]) < separation) {
                    return false;
                }
            }
        }

        return true;
    }
}
