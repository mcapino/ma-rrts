package cz.agents.alite.trajectorytools.util;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

public class SpatialPoint extends Point3d implements cz.agents.alite.vis.element.Point {

    private static final long serialVersionUID = -4209007401452559887L;

    public SpatialPoint(double x, double y, double z) {
        super(x, y, z);
    }

    public SpatialPoint(double[] p) {
        super(p);
    }

    public SpatialPoint(Point3d p1) {
        super(p1);
    }

    public SpatialPoint(Point3f p1) {
        super(p1);
    }

    public SpatialPoint(Tuple3f t1) {
        super(t1);
    }

    public SpatialPoint(Tuple3d t1) {
        super(t1);
    }

    public SpatialPoint() {
        super();
    }

    @Override
    public javax.vecmath.Point3d getPosition() {
        return this;
    }

    public static SpatialPoint interpolate(SpatialPoint p1, SpatialPoint p2, double alpha) {
        SpatialPoint result = new SpatialPoint();
        ((Tuple3d)result).interpolate(p1, p2, alpha);
        return result;
    }



}
