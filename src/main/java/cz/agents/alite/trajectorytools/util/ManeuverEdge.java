package cz.agents.alite.trajectorytools.util;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.planner.spatialmaneuver.PathFindSpecification;
import cz.agents.alite.planner.spatialmaneuver.maneuver.Maneuver;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ManeuverSpecification;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ManeuverSpecification.LevelConstants;
import cz.agents.alite.planner.spatialmaneuver.maneuver.StraightManeuver;
import cz.agents.alite.planner.spatialmaneuver.maneuver.TurnManeuver;

public class ManeuverEdge extends DefaultWeightedEdge {

    private static final long serialVersionUID = 8533824559302072053L;

    private static final LevelConstants DUMMY_MANEUVER_SPECIFICATION_LEVEL_CONSTANTS = new ManeuverSpecification.LevelConstants();
    private static final ManeuverSpecification DUMMY_MANEUVER_SPECIFICATION = new ManeuverSpecification(null, new LevelConstants[] {DUMMY_MANEUVER_SPECIFICATION_LEVEL_CONSTANTS}, null);
    private static final PathFindSpecification DUMMY_PATH_FIND_SPECIFICATION = new PathFindSpecification(0, null, DUMMY_MANEUVER_SPECIFICATION);

    public final Maneuver maneuver;

    private final List<Point3d> pathCache = new ArrayList<Point3d>();
    private double lengthOfPathInCache;

    public static ManeuverEdge createTurnEdge(SpatialPoint start, Vector direction, double radius, double angle) {
        Maneuver turnLeftManeuver = new TurnManeuver(start, direction, 0, radius, angle, DUMMY_PATH_FIND_SPECIFICATION);
        return new ManeuverEdge(turnLeftManeuver );
    }

    public static ManeuverEdge createStraightEdge(SpatialPoint start, Vector direction, double length) {
        Maneuver straightManeuver = new StraightManeuver(start, direction, 0, length, DUMMY_PATH_FIND_SPECIFICATION);
        return new ManeuverEdge(straightManeuver);
    }

    public ManeuverEdge(Maneuver maneuver) {
        this.maneuver = maneuver;
    }

    public OrientedPoint interpolate(double ratio) {
        if (pathCache.isEmpty()) {
            pathCache.add((OrientedPoint) getSource());
            preparePathCache();
        }

        double length = 0;
        Point3d oldPoint = null;
        for (Point3d point : pathCache) {
            if (oldPoint != null) {
                double oldLength = length;
                double currentLength = oldPoint.distance(point);

                length += currentLength;

                if (length > lengthOfPathInCache * ratio) {
                    Vector orientation = new Vector(((OrientedPoint) getSource()).orientation);
                    Vector targetOrientation = ((OrientedPoint) getTarget()).orientation;
                    orientation.interpolate(targetOrientation, ratio);

                    OrientedPoint returnedPoint = new OrientedPoint(new SpatialPoint(oldPoint), orientation);
                    returnedPoint.interpolate(point, (lengthOfPathInCache * ratio - oldLength) / currentLength);
                    return returnedPoint;
                }
            }
            oldPoint = point;
        }

        return null;
    }

    private void preparePathCache() {
        PathManeuverVisitor pathVisitor = new PathManeuverVisitor(pathCache);
        maneuver.accept(pathVisitor);

        lengthOfPathInCache = 0;
        Point3d oldPoint = null;
        for (Point3d point : pathCache) {
            if (oldPoint != null) {
                lengthOfPathInCache += oldPoint.distance(point);
            }
            oldPoint = point;
        }
    }


}
