package cz.agents.alite.trajectorytools.trajectorymetrics;

import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

/**
 *
 * This metric is not symmetric!!!
 *
 * @author honza
 *
 */
public class TrajectoryDistanceMetric<V extends SpatialPoint,E> implements TrajectoryMetric<V,E> {

    public TrajectoryDistanceMetric() {
    }

    @Override
    public double getTrajectoryDistance( PlannedPath<V, E> path, PlannedPath<V, E> otherPath) {

        double distance = 0;
        distance += 0.5 * getVertexToPathDistance(path.getStartVertex(), otherPath);
        distance += 0.5 * getVertexToPathDistance(path.getEndVertex(), otherPath);

        for (E edge : path.getEdgeList()) {
            distance += 0.5 * getVertexToPathDistance(path.getGraph().getEdgeSource(edge), otherPath);
            distance += 0.5 * getVertexToPathDistance(path.getGraph().getEdgeTarget(edge), otherPath);
        }

        return distance;
    }

    private double getVertexToPathDistance(V vertex, PlannedPath<V, E> path) {

        double minDist = Math.min( vertex.distance(path.getStartVertex()), vertex.distance( path.getEndVertex() ));

        for (E edge : path.getEdgeList()) {
            double distance = Math.min( vertex.distance(path.getGraph().getEdgeSource(edge)), vertex.distance( path.getGraph().getEdgeTarget(edge) ));

            minDist = Math.min( minDist, distance );
        }

        return minDist;
    }

    @Override
    public String getName() {
        return "Trajectory Distance";
    }
}
