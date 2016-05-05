package cz.agents.deconfliction.solver.central;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.GraphPathsToEvaluatedTrajectory;

public class ODSolver extends CentralCooperativePathfindingSolver {
    // Ensures optimality
    public static class LowestFFirstComparator implements Comparator<JointODState> {
        @Override
        public int compare(JointODState o1, JointODState o2) {
            int fDiff = (int) Math.signum(o1.f - o2.f);
            if(fDiff == 0) {
                // the two states have the same f value
                int vDiff = o1.v - o2.v;
                if (vDiff == 0) {
                    int hDiff = (int) Math.signum(o1.h - o2.h);
                    return hDiff;
                } else {
                    return vDiff;
                }
            } else {
                return fDiff;
            }
        }
    }

    // Sub-optimal, but focused towards non-conflicting paths
    public static class LowestVFirstComparator implements Comparator<JointODState> {
        @Override
        public int compare(JointODState o1, JointODState o2) {
            int vDiff = o1.v - o2.v;
            if (vDiff == 0) {
                // the two states have the same v value
                int fDiff = (int) Math.signum(o1.f - o2.f);
                if (fDiff == 0) {
                    int hDiff = (int) Math.signum(o1.h - o2.h);
                    return hDiff;
                } else {
                    return fDiff;
                }
            } else {
                return vDiff;
            }
        }
    }

    final long INF = Long.MAX_VALUE;

    JointODState start;
    JointGoalState goal;

    PriorityQueue<JointODState> open;
    Set<JointODState> closed = new HashSet<JointODState>();

    double costLimit;

    List<Trajectory> hardConstrainingTrajectories;
    List<Trajectory> softConstrainingTrajectories;

    long expandedStatesCounter = 0;

    private Comparator<JointODState> comparator;


    public ODSolver(ODState starts[], Waypoint ends[], DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax, Comparator<JointODState> comparator) {
        this(starts, ends, maneuvers, separation, maxTime, vmax, Double.POSITIVE_INFINITY, comparator, new ArrayList<Trajectory>(), new ArrayList<Trajectory>());
    }

    public ODSolver(ODState starts[], Waypoint ends[], DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax) {
        this(starts, ends, maneuvers, separation, maxTime, vmax, Double.POSITIVE_INFINITY, new LowestFFirstComparator(), new ArrayList<Trajectory>(), new ArrayList<Trajectory>());
    }

    public ODSolver(ODState starts[], Waypoint ends[],
            DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation,
            double maxTime, double vmax, double costLimit, Comparator<JointODState> comparator,
            List<Trajectory> hardConstrainingTrajectories,
            List<Trajectory> softConstrainingTrajectories) {

        super(maneuvers, ends, separation, maxTime, vmax);

        if (starts.length != ends.length) {
            throw new IllegalArgumentException(
                    "Start waypoints and end waypoints have different length.");
        }

        this.goal = new JointGoalState(ends);
        this.start = new JointODState(starts, null, this.goal, 0);
        this.hardConstrainingTrajectories = hardConstrainingTrajectories;
        this.softConstrainingTrajectories = softConstrainingTrajectories;
        this.costLimit = costLimit;
        this.maxTime = maxTime;
        this.comparator = comparator;
        this.open = new PriorityQueue<JointODState>(100, comparator);

        open.add(start);
    }


    public GraphPath<Waypoint, SpatialManeuver>[] solve() {
        return searchStep(INF).getPaths();
    }

    public SearchResultGraphPaths searchStep(long timeLimitNs) {

        long interruptAt = System.nanoTime() + timeLimitNs;

        while (!open.isEmpty()) {
            JointODState current = open.poll();

            if (current.equals(goal)) {
                // Found solution - reconstruct paths for all agents
                return new SearchResultGraphPaths(true, reconstructPaths(current));
            }

            closed.add(current);

            List<JointODState> children = current.getChildren(augmentedManeuvers, goal, separation, hardConstrainingTrajectories, softConstrainingTrajectories);
            expandedStatesCounter++;
            Counters.statesExpanded++;

            for (JointODState child : children ) {

                if (!closed.contains(child) && child.getEvaluation() <= costLimit) {
                    updateStateInOpen(child);
                }

            }

            if (System.nanoTime() >= interruptAt && timeLimitNs != INF) {
                return new SearchResultGraphPaths(false, null);
            }
         }

         return new SearchResultGraphPaths(true, null);
    }


    public SearchResult solveTrajectories(long runtimeLimitMs) {
        double[] startTimes = new double[start.nAgents()];
        for (int i = 0; i < startTimes.length; i++) {
            if (start.getAgent(i) != null) {
                startTimes[i] = 0;
                //startTimes[i] = start.getAgent(i).getTime();
            }
        }

        SearchResultGraphPaths result = searchStep(runtimeLimitMs * 1000000);

        if (result.isFinished()) {
            return new SearchResult(GraphPathsToEvaluatedTrajectory.convert(result.getPaths(), startTimes, maxTime), true);
        } else {
            return new SearchResult(null, false);
        }
    }

    private GraphPath<Waypoint, SpatialManeuver>[] reconstructPaths(JointODState goalstate) {
        LinkedList<SpatialManeuver>[] paths = new LinkedList[goalstate.nAgents()];

        for (int i=0; i < goalstate.nAgents(); i++) {
            if (goalstate.getAgent(i) != null) {
                paths[i] = new LinkedList<SpatialManeuver>();
            }
        }

        JointODState current = goalstate;
        while (current.getParent() != null) {
            if (current.isFullyAssigned()) {
                for(int i=0; i < current.nAgents(); i++) {
                    if (current.getAgent(i) != null) {
                        assert(current.getAgent(i).getNextManeuver() != null);
                        paths[i].addFirst(current.getAgent(i).getNextManeuver());
                    }
                }
            }
            current = current.getParent();
        }

        @SuppressWarnings("unchecked")
        GraphPath<Waypoint, SpatialManeuver>[] graphPaths = new GraphPath[goalstate.nAgents()];
        for (int i=0; i < goalstate.nAgents(); i++) {
            if (paths[i] != null) {
                /*double duration = 0.0;
                for(SpatialManeuver maneuver : paths[i]) {
                    duration += maneuver.getDuration();
                } */
                graphPaths[i] = new GraphPathImpl<Waypoint, SpatialManeuver>(augmentedManeuvers[i], start.getAgent(i).getCurrentWaypoint(), goalstate.getAgent(i).getCurrentWaypoint(), paths[i], goalstate.getAgent(i).getCost());
            }
        }

        return graphPaths;
    }

    private void updateStateInOpen(JointODState candidateState) {
        Iterator<JointODState> iterator = open.iterator();
        boolean alreadyContainsBetter = false;

        while (iterator.hasNext()) {
            JointODState state = (JointODState) iterator.next();
            if (state.equals(candidateState)) {
                if (comparator.compare(state,candidateState) <= 0) {
                    alreadyContainsBetter = true;
                } else {
                    iterator.remove();
                }
            }
        }

        if (!alreadyContainsBetter) {
            open.add(candidateState);
        }
    }

    public long getExpandedStatesCounter() {
        return expandedStatesCounter;
    }
}
