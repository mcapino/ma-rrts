package cz.agents.alite.trajectorytools.planner;

public interface GoalPenaltyFunction<V> {

    double getGoalPenalty(V vertex);
}
