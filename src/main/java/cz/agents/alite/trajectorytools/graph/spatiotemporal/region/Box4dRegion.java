package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import javax.vecmath.Point4d;

import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.util.NotImplementedException;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class Box4dRegion implements SpaceTimeRegion {

    Point4d corner1;
    Point4d corner2;

    public Box4dRegion(Point4d corner1, Point4d corner2) {
        super();
        this.corner1 = corner1;
        this.corner2 = corner2;
    }

    @Override
    public boolean intersectsLine(TimePoint p1, TimePoint p2) {
        throw new NotImplementedException();
    }

    @Override
    public boolean isInside(TimePoint p) {
        return ( p.x >= corner1.x && p.x <= corner2.x &&
                 p.y >= corner1.y && p.y <= corner2.y &&
                 p.z >= corner1.z && p.z <= corner2.z &&
                 p.w >= corner1.w && p.w <= corner2.w );

    }

    public Point4d getCorner1() {
        return corner1;
    }

    public Point4d getCorner2() {
        return corner2;
    }

    public double getXSize() {
        return corner2.x - corner1.x;
    }

    public double getYSize() {
        return corner2.y - corner1.y;
    }


    public double getZSize() {
        return corner2.z - corner1.z;
    }

    public double getTSize() {
        return corner2.w - corner1.w;
    }

    public TimePoint getCenter() {
        return new TimePoint(corner1.getX() + getXSize()/2,
                                corner1.getY() + getYSize()/2,
                                corner1.getZ() + getZSize()/2,
                                corner1.getW() + getTSize()/2);
    }

    public BoxRegion get3dBounds() {
        return new BoxRegion(new SpatialPoint(corner1.x, corner1.y, corner1.z),
                new SpatialPoint(corner2.x, corner2.y, corner2.z));
    }

}
