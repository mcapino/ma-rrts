package cz.agents.alite.trajectorytools.trajectorymetrics;

import java.util.Set;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import cz.agents.alite.planner.spatialmaneuver.zone.BoxZone;
import cz.agents.alite.planner.spatialmaneuver.zone.TransformZone;
import cz.agents.alite.planner.spatialmaneuver.zone.Zone;
import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ObstacleAvoidanceMetric<V extends SpatialPoint,E> implements TrajectoryMetric<V,E> {
    private static final int DIRECTIONS = 4;

    @Override
    public double getTrajectoryDistance(
            PlannedPath<V,E> path,
            PlannedPath<V,E> otherPath) {
        
        if (! (path.getGraph() instanceof GraphWithObstacles )) {
            System.out.println(path.getGraph().getClass().getName() + " is not compatible with the Obstacle avoidance metric!");
            return 0;
        }

        Set<? extends SpatialPoint> obstacles = ((GraphWithObstacles<? extends SpatialPoint, E>) path.getGraph()).getObstacles();

        if (obstacles.isEmpty()) {
            return 0;
        }
        
        double similarity = 0;
        
        for (SpatialPoint obstacle : obstacles) {
            for (int direction = 0; direction < DIRECTIONS; direction ++) {
                Zone zone = createZone(obstacle, direction);
                if (isZoneCrossed( zone, path) == isZoneCrossed( zone, otherPath)) {
                    similarity ++;
                }
            }
        }
        
        return 1 - (similarity / (obstacles.size() * DIRECTIONS));
    }

    private boolean isZoneCrossed(Zone zone, PlannedPath<V, E> path) {
        for (E edge : path.getEdgeList()) {
            if (zone.testLine(path.getGraph().getEdgeSource(edge), path.getGraph().getEdgeTarget(edge), null)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String getName() {
        return "Obstacle Avoidance Metric";
    }

    private Zone createZone(SpatialPoint obstacle, int direction) {
        Zone zone;
        Zone boxZone = new BoxZone(new Vector3d(Double.MAX_VALUE, 0.2, 0.2));

        Vector3d translation = new Vector3d(obstacle.x, obstacle.y, obstacle.z - 0.1);
        switch (direction) {
        case 0: // NORTH
            translation.x -= 0.1;
            zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), -Math.PI/2);
            break;
        case 1: // EAST
            translation.y -= 0.1;
            zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), 0);
            break;
        case 2: // SOUTH
            translation.x +=0.1;
            zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), Math.PI/2);
            break;
        case 3: // WEST
            translation.y += 0.1;
            zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), Math.PI);
            break;
        default:
            throw new IllegalArgumentException("Unknown direction: " + direction);
        }

        return zone;
    }
}
