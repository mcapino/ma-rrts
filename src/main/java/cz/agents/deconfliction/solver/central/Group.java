package cz.agents.deconfliction.solver.central;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class Group {
    final ODState[] startAgentStates;
    final Waypoint[] goalWaypoints;
    final int nAgents;
    EvaluatedTrajectory[] trajectories = null;
    private Set<Group> conflictedWith = new HashSet<Group>();
    private double lowerBound = 0.0;

    public Group(ODState[] startAgentStates, Waypoint[] goalWaypoints, double lowerBound) {
        super();
        this.startAgentStates = startAgentStates;
        this.goalWaypoints = goalWaypoints;
        this.nAgents = startAgentStates.length;
        this.lowerBound = lowerBound;
    }

    public SearchResult planOptimalTrajectories(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double separation,
            double maxTime, double vmax, double maxCost,
            List<Trajectory> hardConstrainingTrajectories,
            List<Trajectory> softConstrainingTrajectories, long runtimeLimitMs) {

        ODSolver solver = new ODSolver(startAgentStates, goalWaypoints,
                graph, separation, maxTime, vmax, maxCost, new ODSolver.LowestFFirstComparator(),
                hardConstrainingTrajectories, softConstrainingTrajectories);

        SearchResult result = solver.solveTrajectories(runtimeLimitMs);

        if (result.isFinished()) {

            if (result.getTrajectories() != null) {
                trajectories = result.getTrajectories();
                lowerBound = getCost();
            } else {
                trajectories = null;
            }
        }

        return result;
    }

    public SearchResult planAnyTrajectories(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double separation,
            double maxTime, double vmax,
            List<Trajectory> hardConstrainingTrajectories,
            List<Trajectory> softConstrainingTrajectories,
            long runtimeLimitMs) {
        ODSolver solver = new ODSolver(startAgentStates, goalWaypoints,
                graph, separation, maxTime, vmax, Double.POSITIVE_INFINITY, new ODSolver.LowestVFirstComparator(),
                hardConstrainingTrajectories, softConstrainingTrajectories);

        SearchResult result = solver.solveTrajectories(runtimeLimitMs);

        if (result.isFinished()) {
            trajectories = result.getTrajectories();
        }

        return result;
    }

    public EvaluatedTrajectory[] getTrajectories() {
        return trajectories;
    }

    public boolean containsAgent(int i) {
        return (startAgentStates[i] != null);
    }

    public static Group join(Group g1, Group g2) {
        assert(g1.nAgents == g2.nAgents);
        ODState[] startAgentStates = new ODState[g1.nAgents];
        Waypoint[] goalWaypoints = new Waypoint[g1.nAgents];

        for (int i=0; i < g1.nAgents; i++) {
            if (g1.startAgentStates[i] != null) {
                startAgentStates[i] = g1.startAgentStates[i];
            } else if (g2.startAgentStates[i] != null) {
                startAgentStates[i] = g2.startAgentStates[i];
            }
        }

        for (int i=0; i < g1.nAgents; i++) {
            if (g1.goalWaypoints[i] != null) {
                goalWaypoints[i] = g1.goalWaypoints[i];
            } else if (g2.startAgentStates[i] != null) {
                goalWaypoints[i] = g2.goalWaypoints[i];
            }
        }

        return new Group(startAgentStates, goalWaypoints, g1.getLowerBound() + g2.getLowerBound());
    }

    @Override
    public String toString() {
        return getShortString();
    }

    public String getLongString() {
        return "group(" + Arrays.toString(startAgentStates) + ")";
    }

    public String getShortString() {
        StringBuilder sb = new StringBuilder();
        sb.append("G([");
        for(int i=0; i<startAgentStates.length; i++) {
            if (startAgentStates[i] != null) {
                sb.append(i);

                sb.append(", ");
            }
        }
        //removes the last ", "
        sb.deleteCharAt(sb.length()-1);
        sb.deleteCharAt(sb.length()-1);

        if (trajectories != null) {
            sb.append(String.format("] lb: %.2f cost: %.2f", lowerBound, getCost()));
        } else {
            sb.append(String.format("] lb: %.2f cost: N/A", lowerBound));
        }

        sb.append(")");

        return sb.toString();
    }



    public void revertTo(EvaluatedTrajectory[] originalG1Trajectories) {
        trajectories = originalG1Trajectories;
    }

    public void setConflictedWith(Group g2) {
        conflictedWith.add(g2);
    }

    public boolean conflictedBeforeWith(Group g2) {
        return conflictedWith.contains(g2);
    }

    public double getCost() {
        double cost = 0.0;
        for (EvaluatedTrajectory trajectory : trajectories) {
            if (trajectory != null) {
                cost += trajectory.getCost();
            }
        }
        return cost;
    }

    public int getSize() {
        int count = 0;
        for (ODState startAgentState : startAgentStates) {
            if (startAgentState != null) {
                count++;
            }
        }
        return count;
    }

    public double getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(double lowerBound) {
        this.lowerBound = lowerBound;
    }

    public void resetConflictedBefore() {
        conflictedWith = new HashSet<Group>();
    }
}
