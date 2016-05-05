package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.graph.spatial.region.SphereRegion;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class StaticSphereRegion extends SphereRegion implements SpaceTimeRegion {

    public StaticSphereRegion(Point3d center, double radius) {
        super(center, radius);
    }

    @Override
    public boolean intersectsLine(TimePoint p1, TimePoint p2) {
        return findLineSphereIntersection(p1.getSpatialPoint(), p2.getSpatialPoint(), center, radius) != null;
    }

    @Override
    public boolean isInside(TimePoint p) {
        return center.distance(p.getSpatialPoint()) <= radius;
    }

}
