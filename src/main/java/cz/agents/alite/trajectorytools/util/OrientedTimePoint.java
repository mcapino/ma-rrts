package cz.agents.alite.trajectorytools.util;

import javax.vecmath.Matrix4d;
import javax.vecmath.Tuple4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class OrientedTimePoint extends TimePoint {
    private static final long serialVersionUID = 5757663724715260037L;

    // A vector representing the orientation
    public Vector3d orientation;
    
    public OrientedTimePoint(double x, double y, double z, double t, double dx, double dy, double dz) {
        this(new TimePoint(x,y,z,t), new Vector3d(dx,dy,dz));
    }
    
    /**
     * Creates an oriented point from a timepoint and yaw rotation. 
     * Assuming coordinate systems, where x-axis goes from noth to southe, 
     * where higher values are more south, and y-axis goes from east to west.
     * 
     * @param yaw cw rotation in z-axis, 0 is north, PI/2 is east and  -PI/2 (or 3/2PI) is west.
     */
    public OrientedTimePoint(TimePoint point, double yaw) {
        this(point, new Vector3d(Math.sin(yaw), Math.cos(yaw), 0)); 
    }
    
    public OrientedTimePoint(TimePoint point, Vector3d orientation) {
        super(point);
        if (orientation == null) {
            throw new NullPointerException("Orientation cannot be null.");
        }
        this.orientation = new Vector(orientation);
    }

    public OrientedTimePoint(OrientedTimePoint orientedPoint) {
        super(orientedPoint);
        this.orientation = new Vector(orientedPoint.orientation);
    }

    @Override
    public int hashCode() {
        return super.hashCode() ^ orientation.hashCode();
    }

    @Override
    public boolean equals(Object other) {
        try {
            return super.equals(other) && orientation.equals(((OrientedTimePoint) other).orientation);
        } catch (ClassCastException e) {
            return false;
        }
    }

    @Override
    public String toString() {
        return super.toString() + "x" + orientation;
    }

    public void rotate(double angleInRads) {
        Matrix4d rotationMatrix = new Matrix4d();
        rotationMatrix.rotZ(angleInRads);
        rotationMatrix.transform(this.orientation);
    }

    public OrientedTimePoint getOppositeDirectionPoint() {
        Vector newOrientation = new Vector(orientation);
        newOrientation.x *= -1;
        newOrientation.y *= -1;

        return new OrientedTimePoint(this, newOrientation);
    }

      public double distanceInclOrientation(OrientedTimePoint p1)
      {
        double dx, dy, dz, dw, ddx, ddy, ddz;

        dx = this.x-p1.x;
        dy = this.y-p1.y;
        dz = this.z-p1.z;
        dw = this.w-p1.w;

        ddx = this.orientation.x - p1.orientation.x;
        ddy = this.orientation.y - p1.orientation.y;
        ddz = this.orientation.z - p1.orientation.z;

        return Math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw + 10*ddx*ddx + 10*ddy*ddy + 10*ddz*ddz);
      }

	public boolean epsilonEquals(OrientedTimePoint other, double epsilon) {
		return super.epsilonEquals(other, epsilon) && orientation.epsilonEquals(other.orientation, epsilon);
	}
	
    /**
     * An angle between two vectors, adapted the implementation from VecMath to return angle in range [-PI and PI]
     */
    public static double angle(Vector2d a, Vector2d b)
    {

       double vDot = a.dot(b) / ( a.length()*b.length() );
       if( vDot < -1.0) vDot = -1.0;
       if( vDot >  1.0) vDot =  1.0;

       double angle = Math.atan2( a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y );

       return(angle);
    }
    
    public OrientedPoint getOrientedPoint() {
    	return new OrientedPoint(this.getSpatialPoint(), new Vector(orientation));
    }
	
      
      
}
