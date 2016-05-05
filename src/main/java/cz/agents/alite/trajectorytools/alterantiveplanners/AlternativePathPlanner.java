package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.util.Collection;

import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.planner.PlannedPath;

public interface AlternativePathPlanner<V,E> {

    Collection<PlannedPath<V, E>> planPath(GraphWithObstacles<V,E> graph, V startVertex, V endVertex);

    String getName();
}