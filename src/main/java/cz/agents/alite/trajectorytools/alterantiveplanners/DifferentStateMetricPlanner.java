package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.planner.GoalPenaltyFunction;
import cz.agents.alite.trajectorytools.planner.PathPlanner;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.planner.SingleVertexPlannedPath;
import cz.agents.alite.trajectorytools.trajectorymetrics.DifferentStateMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectoryMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectorySetMetrics;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class DifferentStateMetricPlanner<V extends SpatialPoint, E> implements AlternativePathPlanner<V,E> {

    private static final int ALPHA = 5;

    private final PathPlanner<V,E> planner;

    private final int pathSolutionLimit;

    private final TrajectoryMetric<V, E> metric;

    public DifferentStateMetricPlanner(PathPlanner<V,E> planner, int pathSolutionLimit) {
        this.planner = planner;
        this.pathSolutionLimit = pathSolutionLimit;

        metric = new DifferentStateMetric<V, E>();
    }

    @Override
    public Collection<PlannedPath<V, E>> planPath(final GraphWithObstacles<V, E> graph, V startVertex, V endVertex) {
        final List<PlannedPath<V, E>> paths = new ArrayList<PlannedPath<V, E>>(pathSolutionLimit);

        for (int i = 0; i < pathSolutionLimit; i++) {
            PlannedPath<V, E> path = planner.planPath(
                    graph, startVertex, endVertex,
                    new GoalPenaltyFunction<V>() {
                        @Override
                        public double getGoalPenalty(V vertex) {
                            double value = TrajectorySetMetrics.getRelativePlanSetAvgDiversity(
                                    new SingleVertexPlannedPath<V,E>(graph, vertex),
                                    paths,
                                    metric
                                    );
                            return ALPHA * (1 - value);
                        }
                    },
                    planner.getHeuristicFunction()
                    );
            if (path == null) {
                System.out.println("!!!!!! NULL !!!!!!!");
            }
            paths.add( path );
        }

        return paths;
    }

    @Override
    public String getName() {
        return "Different States Metric Planner";
    }
}
