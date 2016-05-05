package cz.agents.alite.trajectorytools.planner;

public class NullGoalPenaltyFunction<V> implements GoalPenaltyFunction<V> {
    @Override
    public double getGoalPenalty(V state) {
        return 0;
    }
}