package cz.agents.alite.trajectorytools.util;

import java.util.Random;

import javax.vecmath.Matrix4d;



/**
 * A point having a specific orientation.
 */
public class OrientedPoint extends SpatialPoint {
    private static final long serialVersionUID = 5757663724715260037L;

    // A normalized vector representing the orientation
    public Vector orientation;


    public OrientedPoint(double x, double y, double z, double u, double v, double w) {
        this(new SpatialPoint(x,y,z), new Vector(u,v,w));
    }

    public OrientedPoint(SpatialPoint point, Vector orientation) {
        super(point);
        if (orientation == null) {
            throw new NullPointerException("Orientation cannot be null.");
        }
        this.orientation = new Vector(orientation);
    }

    public OrientedPoint(OrientedPoint orientedPoint) {
        super(orientedPoint);
        this.orientation = new Vector(orientedPoint.orientation);
    }

    @Override
    public int hashCode() {
        return super.hashCode() + orientation.hashCode();
    }

    @Override
    public boolean equals(Object other) {
        try {
            return super.equals(other) && orientation.equals(((OrientedPoint) other).orientation);
        } catch (ClassCastException e) {
            return false;
        }
    }

    @Override
    public String toString() {
        return super.toString() + "x" + orientation;
    }

    /**
     * Calculates angle of vector
     * @return Double-precision angle
     */
    public double getAngle()
    {
        double angle;
        double x = orientation.x;
        double y = orientation.y;

        angle=Math.atan2(y, x)*180.0/Math.PI;
        if (angle < 0.0)
            angle = 360.0 + angle;

        return angle;

    }

    /**
     * Generates a random point within a given radius from this point.
     */
    public OrientedPoint getNearRandomPoint(Random random, double radius) {
        double nx;
        double ny;

        double theta = random.nextDouble() * 2 * Math.PI;
        double r = random.nextDouble() * radius;

        nx = r * Math.cos(theta);
        ny = r * Math.sin(theta);

        return new OrientedPoint(new SpatialPoint(x + nx, y + ny, z), orientation);
    }

    public void rotate(double angleInRads) {
        Matrix4d rotationMatrix = new Matrix4d();
        rotationMatrix.rotZ(angleInRads);
        rotationMatrix.transform(this.orientation);
    }

    public OrientedPoint getOppositeDirectionPoint() {
        Vector newOrientation = new Vector(orientation);
        newOrientation.x *= -1;
        newOrientation.y *= -1;

        return new OrientedPoint(this, newOrientation);
    }

}
