package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class AlternativePlannerSelector<V extends SpatialPoint,E> implements AlternativePathPlanner<V,E> {

    private final AlternativePathPlanner<V,E> planner;
    private final int limit;

    public AlternativePlannerSelector(AlternativePathPlanner<V,E> planner, int limit) {
        this.planner = planner;
        this.limit = limit;
    }

    @Override
    public Collection<PlannedPath<V, E>> planPath(
            GraphWithObstacles<V,E> graph, V startVertex,
            V endVertex) {

        Collection<PlannedPath<V, E>> plannedPaths = planner.planPath(graph, startVertex, endVertex);

        return getShortestPaths(plannedPaths, limit);
    }

    static public <V, E> Collection<PlannedPath<V, E>> getShortestPaths(
            Collection<PlannedPath<V, E>> plannedPaths, int limit) {
        if (plannedPaths.size() <= limit) {
            return plannedPaths;
        }

        List<PlannedPath<V, E>> paths = new ArrayList<PlannedPath<V,E>>(plannedPaths);

        Collections.sort(paths, new Comparator<PlannedPath<V, E>>() {
            @Override
            public int compare(PlannedPath<V, E> o1,
                    PlannedPath<V, E> o2) {
                return Double.compare(o1.getWeight(), o2.getWeight() );
            }
        });

        return paths.subList(0, limit);
    }

    @Override
    public String getName() {
        return planner.getName() + " - limit (" + limit + ")";
    }

}
