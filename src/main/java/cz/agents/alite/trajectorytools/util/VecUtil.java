package cz.agents.alite.trajectorytools.util;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class VecUtil {

    public static Point2d horizontal(Point3d p) {
        return new Point2d(p.x, p.y);
    }

    public static Vector2d horizontal(Vector3d p) {
        return new Vector2d(p.x, p.y);
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

    public static Vector2d rotateYaw(Vector2d vector, double yawInRad)
    {
        double rx = (vector.x * Math.cos(yawInRad)) - (vector.y * Math.sin(yawInRad));
        double ry = (vector.x * Math.sin(yawInRad)) + (vector.y * Math.cos(yawInRad));
        return new Vector2d(rx, ry);
    }
}
