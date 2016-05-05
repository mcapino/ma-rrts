package cz.agents.alite.trajectorytools.trajectory;

/**
 * A sampling-based trajectory approximation.
 */
public class EvaluatedSampledTrajectory extends SampledTrajectory implements EvaluatedTrajectory{

    double cost;
    public EvaluatedSampledTrajectory(EvaluatedTrajectory continuousTrajectory, double samplingInterval) {
        super(continuousTrajectory, samplingInterval);
        this.cost = continuousTrajectory.getCost();
    }

    @Override
    public double getCost() {
        return cost;
    }
}
