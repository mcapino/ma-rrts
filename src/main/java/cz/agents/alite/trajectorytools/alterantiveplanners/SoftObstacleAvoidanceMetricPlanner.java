package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.util.Collection;

import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

/**
 * Not working - cannot be planned by AStar type of planner!
 *
 * Can propose 3 trajectories at maximum...
 *
 * Price of the edge depends on the previous trajectory
 *
 * @author honza
 *
 */
public class SoftObstacleAvoidanceMetricPlanner<V extends SpatialPoint, E> implements AlternativePathPlanner<V,E> {

    public SoftObstacleAvoidanceMetricPlanner() {
    }

    @Override
    public Collection<PlannedPath<V, E>> planPath(GraphWithObstacles<V,E> graph, V startVertex, V endVertex) {
        throw new UnsupportedOperationException("cannot be planned by AStar type of planner!");
    }

    @Override
    public String getName() {
        return "Soft Obstacle Avoidance";
    }
}