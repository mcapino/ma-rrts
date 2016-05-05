package cz.agents.deconfliction.solver.central;

import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;

public interface Listener {
    public abstract void notifyNewSolution(EvaluatedTrajectory[] trajectories, boolean provedOptimal);
}
