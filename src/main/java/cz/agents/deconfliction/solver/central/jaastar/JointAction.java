package cz.agents.deconfliction.solver.central.jaastar;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointAction {
    SpatialManeuver maneuvers[];
    Waypoint[] targets;
    double[] costs;

    public JointAction(SpatialManeuver[] maneuvers, Waypoint[] targets, double[] costs) {
        super();
        this.maneuvers = maneuvers;
        this.targets = targets;
        this.costs = costs;
    }

    public static JointAction addActionForAgent(JointAction action, int i,
            SpatialManeuver maneuver, Waypoint target, double cost) {
        SpatialManeuver[] newManeuvers = Arrays.copyOf(action.maneuvers, action.maneuvers.length);
        Waypoint[] newTargets = Arrays.copyOf(action.targets, action.targets.length);
        double[] newCosts = Arrays.copyOf(action.costs, action.costs.length);

        newManeuvers[i] = maneuver;
        newTargets[i] = target;
        newCosts[i] = cost;

        return new JointAction(newManeuvers, newTargets, newCosts);
    }

    public Trajectory[] getTrajectories(double time) {
        Trajectory[] trajectories = new Trajectory[maneuvers.length];
        for (int i=0; i<maneuvers.length; i++) {
            if (maneuvers[i] != null) {
                trajectories[i] = maneuvers[i].getTrajectory(time);
            }
        }

        return trajectories;
    }

    public List<Trajectory> getTrajectoryList(double startTime) {
        List<Trajectory> trajectories = new LinkedList<Trajectory>();
        for (int i=0; i<maneuvers.length; i++) {
            if (maneuvers[i] != null) {
                trajectories.add(maneuvers[i].getTrajectory(startTime));
            }
        }

        return trajectories;
    }

    public SpatialManeuver[] getManeuvers() {
        return maneuvers;
    }

    public Waypoint[] getTargets() {
        return targets;
    }

    public double[] getCosts() {
        return costs;
    }

    public double getCost() {
        double cost = 0;
        for (int i = 0; i < costs.length; i++) {
            cost += costs[i];
        }
        return cost;
    }

    @Override
    public String toString() {
        return "JointAction [maneuvers=" + Arrays.toString(maneuvers)
                + ", targets=" + Arrays.toString(targets) + ", costs="
                + Arrays.toString(costs) + "]";
    }


}
