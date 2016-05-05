package cz.agents.alite.trajectorytools.util;

import javax.vecmath.Point3d;
import javax.vecmath.Point4d;

public class TimePoint extends Point4d {
    private static final long serialVersionUID = 1136064568843307511L;
    private static final double EPSILON = 1e-6;

    public TimePoint(double x, double y, double z, double time) {
        super(x,y,z,time);
        
        if(time == Double.NaN){
        	throw new IllegalArgumentException("Time must be a number! " + time);
        }
    }

    public TimePoint(Point3d p, double time) {
        super(p.x,p.y,p.z,time);
    }

    public TimePoint(Point4d p1) {
        super(p1);
    }

    public SpatialPoint getSpatialPoint() {
        return new SpatialPoint(x,y,z);
    }

    public double getTime() {
        return w;
    }

    @Override
    public String toString() {
        return super.toString();
    }

}
