package cz.agents.deconfliction.solver.central;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.FreeWhenOnTargetGraph;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.Trajectories;

public class CentralCooperativePathfindingSolver {

    protected DirectedGraph<Waypoint, SpatialManeuver>[] augmentedManeuvers;

    protected Waypoint[] targets;

    protected double separation;
    protected double maxTime;
    protected double vmax;
    protected int nAgents;

    protected final double SAMPLES_PER_SEPARATION = 4.0;

    List<Listener> listeners = new LinkedList<Listener>();


    @SuppressWarnings("unchecked")
    protected CentralCooperativePathfindingSolver(DirectedGraph<Waypoint, SpatialManeuver> maneuvers,
            Waypoint[] targets,
            double separation,
            double maxTime,
            double vmax) {
        super();
        this.targets = targets;
        this.nAgents = targets.length;

        this.augmentedManeuvers = new DirectedGraph[nAgents];
        for (int i=0; i<nAgents; i++) {
            this.augmentedManeuvers[i]
                = new FreeWhenOnTargetGraph<Waypoint, SpatialManeuver>(maneuvers, targets[i]);
        }

        this.separation = separation;
        this.maxTime = maxTime;
        this.vmax = vmax;
    }

    /**
     * Return trajectories from all groups in the list, ignoring the ones given in excludeGroup arguments
     */
    static List<Trajectory> generateSoftConstrainingTrajectories(List<Group> groups, Group... excludeGroup) {
        List<Trajectory> trajectories = new LinkedList<Trajectory>();
        Set<Group> exclude = new HashSet<Group>(Arrays.asList(excludeGroup));

        for(Group group : groups) {
            if (!exclude.contains(group)) {
                trajectories.addAll(toList(group.getTrajectories()));
            }
        }
        return trajectories;
    }

    /**
     * Adds (and overwrites) all non-null elements from array b to array a.
     */
    static protected EvaluatedTrajectory[] add(EvaluatedTrajectory[] a, EvaluatedTrajectory[] b){
        return Trajectories.add(a, b);
    }

    static protected List<Trajectory> toList(EvaluatedTrajectory[] trajectories) {
        return Trajectories.toList(trajectories);
    }

    protected void notifyNewSolution(EvaluatedTrajectory[] trajectories, boolean provedOptimal) {
        for (Listener listener : listeners) {
            EvaluatedTrajectory[] trajectoriesCopy = Arrays.copyOf(trajectories, trajectories.length);
            listener.notifyNewSolution(trajectoriesCopy, provedOptimal);
        }
    }

    public void registerListener(Listener listener) {
        listeners.add(listener);
    }
}
