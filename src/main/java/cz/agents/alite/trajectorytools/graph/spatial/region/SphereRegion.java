package cz.agents.alite.trajectorytools.graph.spatial.region;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class SphereRegion implements SpaceRegion {

    protected Point3d center;
    protected double radius;

    public SphereRegion(Point3d center, double radius) {
        super();
        this.center = center;
        this.radius = radius;
    }

    @Override
    public boolean intersectsLine(SpatialPoint p1, SpatialPoint p2) {
        return findLineSphereIntersection(p1, p2, center, radius) != null;
    }

    @Override
    public boolean isInside(SpatialPoint p) {
        return center.distance(p) <= radius;
    }
    /**
     * Find an intersection between a line segment and a sphere. Return a waypoint which lies in the center of line segment defined by the
     * intersection points, if there are any. If there is no intersection, or the line is a tangent of the sphere, return null.
     */
    public SpatialPoint findLineSphereIntersection(Point3d from, Point3d to, Point3d sphereCenter, double radius){
        Point3d p = from;
        Point3d q = to;

        double dx = q.x - p.x;
        double dy = q.y - p.y;
        double dz = q.z - p.z;

        double a = dx*dx + dy*dy + dz*dz;

        double b = 2*dx*(p.x-sphereCenter.x) +  2*dy*(p.y-sphereCenter.y) +  2*dz*(p.z-sphereCenter.z);

        double c = sphereCenter.x*sphereCenter.x + sphereCenter.y*sphereCenter.y + sphereCenter.z*sphereCenter.z + p.x*p.x + p.y*p.y + p.z*p.z +

                                -2*(sphereCenter.x*p.x + sphereCenter.y*p.y + sphereCenter.z*p.z) - radius*radius;

        double determinant = b * b - 4 * a * c ;

        if(determinant > 0){
            double t1 = (-b + Math.sqrt(determinant)) / (2*a);
            double t2 = (-b - Math.sqrt(determinant)) / (2*a);

            if(t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1){

                SpatialPoint P1 = new SpatialPoint(
                        p.x + t1 * dx,
                        p.y + t1 * dy,
                        p.z + t1 * dz);

                SpatialPoint P2 = new SpatialPoint(
                        p.x + t2 * dx,
                        p.y + t2 * dy,
                        p.z + t2 * dz);

                if(P1.distance(P2) > 0.00){

                    SpatialPoint P = new SpatialPoint(
                            (p.x + t1 * dx + p.x + t2 * dx)/2,
                            (p.y + t1 * dy + p.y + t2 * dy)/2,
                            (p.z + t1 * dz + p.z + t2 * dz)/2);


                    return new SpatialPoint(P);
                }
            }
        }

        return null;
    }

    public Point3d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }
}
