package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.Collection;
import java.util.Random;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.Straight;
import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class SpatialStraightLineDomain implements Domain<SpatialPoint, SpatialManeuver> {

    BoxRegion bounds;
    Collection<SpaceRegion> obstacles;
    SpaceRegion target;
    Random random;
    double speed;

    public SpatialStraightLineDomain(BoxRegion bounds, Collection<SpaceRegion> obstacles, SpaceRegion target, double speed) {
        super();
        this.bounds = bounds;
        this.obstacles = obstacles;
        this.target = target;
        this.random = new Random(1);
        this.speed = speed;
    }

    @Override
    public SpatialPoint sampleState() {
        SpatialPoint point;
        do {
            double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
            double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
            double z = bounds.getCorner1().z + (random.nextDouble() * (bounds.getCorner2().z - bounds.getCorner1().z));
            point = new SpatialPoint(x, y, z);
        } while (!isInFreeSpace(point));
        return point;
    }

    @Override
    public Extension<SpatialPoint, SpatialManeuver>
    extendTo(SpatialPoint from, SpatialPoint to) {
        Extension<SpatialPoint, SpatialManeuver> result = null;
        if (isVisible(from, to)) {
            Straight maneuver = new Straight(from, to, speed);
            result = new Extension<SpatialPoint, SpatialManeuver>(from, to, maneuver, maneuver.getDuration(), true);
        }
        return result;
    }

    private boolean isVisible(SpatialPoint p1, SpatialPoint p2) {

        // check obstacles
        for (SpaceRegion obstacle : obstacles) {
            if (obstacle.intersectsLine(p1, p2)) {
                return false;
            }
        }
        return true;
    }

    protected boolean isInFreeSpace(SpatialPoint p) {
        if (bounds.isInside(p)) {
            for (SpaceRegion obstacle : obstacles) {
                if (obstacle.isInside(p)) {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    @Override
    public ExtensionEstimate estimateExtension(SpatialPoint p1, SpatialPoint p2) {
        return new ExtensionEstimate(p1.distance(p2)/speed, true);
    }

    @Override
    public double distance(SpatialPoint p1, SpatialPoint p2) {
        return p1.distance(p2);
    }

    @Override
    public double nDimensions() {
        return 2;
    }

    @Override
    public boolean isInTargetRegion(SpatialPoint p) {
        return target.isInside(p);
    }

    @Override
    public double estimateCostToGo(SpatialPoint p1) {
        return 0.0;
    }
}
