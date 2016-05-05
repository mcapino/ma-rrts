package cz.agents.alite.trajectorytools.vis.projection;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

public class DefaultProjection<P extends Point3d> implements ProjectionTo2d<P> {

    @Override
    public Point2d project(Point3d point) {
        return new Point2d(point.x, point.y);
    }

}
