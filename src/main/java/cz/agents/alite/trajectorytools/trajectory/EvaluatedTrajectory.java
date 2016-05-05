package cz.agents.alite.trajectorytools.trajectory;


/**
 * A trajectory having a certain cost.
 */
public interface EvaluatedTrajectory extends Trajectory {
    public double getCost();
}
