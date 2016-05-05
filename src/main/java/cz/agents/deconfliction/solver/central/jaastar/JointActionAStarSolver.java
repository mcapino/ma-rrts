package cz.agents.deconfliction.solver.central.jaastar;

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
import cz.agents.deconfliction.solver.central.CentralCooperativePathfindingSolver;
import cz.agents.deconfliction.solver.central.Counters;
import cz.agents.deconfliction.solver.central.SearchResult;
import cz.agents.deconfliction.solver.central.SearchResultGraphPaths;
import cz.agents.deconfliction.solver.central.jaastar.JointState.TimeoutExceededException;
import cz.agents.deconfliction.util.GraphPathsToEvaluatedTrajectory;

public class JointActionAStarSolver extends CentralCooperativePathfindingSolver {
    // Ensures optimality
    public static class LowestFFirstComparator implements Comparator<JointState> {
        @Override
        public int compare(JointState o1, JointState o2) {
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
    public static class LowestVFirstComparator implements Comparator<JointState> {
        @Override
        public int compare(JointState o1, JointState o2) {
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

    JointState start;
    Waypoint[] goal;

    PriorityQueue<JointState> open ;
    Set<JointState> closed = new HashSet<JointState>();

    double costLimit;

    List<Trajectory> hardConstrainingTrajectories;
    List<Trajectory> softConstrainingTrajectories;

    long expandedStatesCounter = 0;

    private Comparator<JointState> comparator;


    public JointActionAStarSolver(Waypoint[] starts, Waypoint[] ends, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax, Comparator<JointState> comparator) {
        this(starts, ends, maneuvers, separation, maxTime, vmax, Double.POSITIVE_INFINITY, comparator, new ArrayList<Trajectory>(), new ArrayList<Trajectory>());
    }

    public JointActionAStarSolver(Waypoint[] starts, Waypoint[] ends, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax) {
        this(starts, ends, maneuvers, separation, maxTime, vmax, Double.POSITIVE_INFINITY, new LowestFFirstComparator(), new ArrayList<Trajectory>(), new ArrayList<Trajectory>());
    }

    public JointActionAStarSolver(Waypoint[] starts, Waypoint[] ends,
            DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation,
            double maxTime, double vmax, double costLimit, Comparator<JointState> comparator,
            List<Trajectory> hardConstrainingTrajectories,
            List<Trajectory> softConstrainingTrajectories) {

        super(maneuvers, ends, separation, maxTime, vmax);

        if (starts.length != ends.length) {
            throw new IllegalArgumentException(
                    "Start waypoints and end waypoints have different length.");
        }

        this.goal = ends;
        this.start = new JointState(starts, 0.0, null, null, 0, this.goal);
        this.hardConstrainingTrajectories = hardConstrainingTrajectories;
        this.softConstrainingTrajectories = softConstrainingTrajectories;
        this.costLimit = costLimit;
        this.maxTime = maxTime;
        this.comparator = comparator;
        this.open = new PriorityQueue<JointState>(100, comparator);

        open.add(start);
    }


    public SearchResultGraphPaths solve(long runtimeLimitMs) {
        return searchStep(runtimeLimitMs * 1000000);
    }

    public SearchResultGraphPaths searchStep(long timeLimitNs) {

        long interruptAtNs = System.nanoTime() + timeLimitNs;
        try {
	        while (!open.isEmpty()) {
	            JointState current = open.poll();
	
	            if (current.areAgentsAt(goal)) {
	                // Found solution - reconstruct paths for all agents
	                return new SearchResultGraphPaths(true, reconstructPaths(current));
	            }
	
	            closed.add(current);
	            
	            List<JointState> children = current.getChildren(augmentedManeuvers, goal, separation, hardConstrainingTrajectories, softConstrainingTrajectories, interruptAtNs);
	            expandedStatesCounter++;
	            Counters.statesExpanded++;
	
	            for (JointState child : children ) {
	            	
		            if (System.nanoTime() >= interruptAtNs && timeLimitNs != INF) {
		                return new SearchResultGraphPaths(false, null);
		            }
	
	                if (!closed.contains(child) && child.getEvaluation() <= costLimit) {
	                    updateStateInOpen(child);
	                }
	
	            }
	         }
        } catch (TimeoutExceededException e) {
        	return new SearchResultGraphPaths(false, null);
        } 

         return new SearchResultGraphPaths(true, null);
    }


    public SearchResult solveTrajectories(long timeoutMs) {
        double[] startTimes = new double[start.nAgents()];
        for (int i = 0; i < startTimes.length; i++) {
            if (start.getAgent(i) != null) {
                startTimes[i] = 0;
            }
        }

        SearchResultGraphPaths result = solve(timeoutMs);

        if (result.isFinished()) {
            return new SearchResult(GraphPathsToEvaluatedTrajectory.convert(result.getPaths(), startTimes, maxTime), true);
        } else {
            return new SearchResult(null, false);
        }
    }

    private GraphPath<Waypoint, SpatialManeuver>[] reconstructPaths(JointState goalstate) {
        LinkedList<SpatialManeuver>[] paths = new LinkedList[goalstate.nAgents()];
        double[] costs = new double[goalstate.nAgents()];

        for (int i=0; i < goalstate.nAgents(); i++) {
            if (goalstate.getAgent(i) != null) {
                paths[i] = new LinkedList<SpatialManeuver>();
            }
        }

        JointState current = goalstate;
        while (current.getParent() != null) {
                for(int i=0; i < current.nAgents(); i++) {
                    if (current.getAgent(i) != null) {
                        paths[i].addFirst(current.getEdgeFromParent().getManeuvers()[i]);
                        costs[i] += current.getEdgeFromParent().getCosts()[i];
                    }
            }
            current = current.getParent();
        }

        @SuppressWarnings("unchecked")
        GraphPath<Waypoint, SpatialManeuver>[] graphPaths = new GraphPath[goalstate.nAgents()];
        for (int i=0; i < goalstate.nAgents(); i++) {
            if (paths[i] != null) {
                graphPaths[i] = new GraphPathImpl<Waypoint, SpatialManeuver>(
                        augmentedManeuvers[i], start.getAgent(i),
                        goalstate.getAgent(i), paths[i],
                        costs[i]);
           }
        }

        return graphPaths;
    }

    private void updateStateInOpen(JointState candidateState) {
        Iterator<JointState> iterator = open.iterator();
        boolean alreadyContainsBetter = false;

        while (iterator.hasNext()) {
            JointState state = (JointState) iterator.next();
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
