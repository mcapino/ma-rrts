package cz.agents.alite.trajectorytools.trajectory;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Vector;

public class LoiteringTrajectory implements Trajectory {
	
	public enum LoiteringDirection { CLOCKWISE, ANTI_CLOCKWISE }
	
	final Point3d loiteringOrigin;
	final OrientedPoint startPoint;
	final double startTime;
	final double loiteringRadius;
	final LoiteringDirection direction;
	final double loiteringSpeed;
	
	public LoiteringTrajectory(Point3d loiteringOrigin, OrientedPoint startPoint,double startTime, double loiteringRadius, LoiteringDirection direction, double loiteringSpeed){
		this.loiteringOrigin = loiteringOrigin;
		this.startPoint = startPoint;
		this.startTime = startTime;
		this.loiteringRadius = loiteringRadius;
		this.direction = direction;
		this.loiteringSpeed = loiteringSpeed;
	}

	@Override
	public double getMinTime() {
		return startTime;
	}

	@Override
	public double getMaxTime() {
		return Double.POSITIVE_INFINITY;
	}

	@Override
	public OrientedPoint getPosition(double t) {

		double travelledAngle = loiteringSpeed * (t-startTime) / loiteringRadius;
		
		double x = loiteringOrigin.x+(startPoint.x - loiteringOrigin.x) * Math.cos(travelledAngle) - (startPoint.y - loiteringOrigin.y) * Math.sin(travelledAngle);
		double y = loiteringOrigin.y+(startPoint.x - loiteringOrigin.x) * Math.sin(travelledAngle) + (startPoint.y - loiteringOrigin.y) * Math.cos(travelledAngle);
		
		Point3d pos = new Point3d(x,y,loiteringOrigin.z);
		
		//orientation
		double ca = Math.cos(travelledAngle);
		double sa = Math.sin(travelledAngle);
		
		Vector orientation = new Vector(startPoint.orientation.x * ca - startPoint.orientation.y * sa, startPoint.orientation.x * sa + startPoint.orientation.y * ca, startPoint.orientation.z);
		
		return new OrientedPoint(new SpatialPoint(pos),orientation);
	}
	
	public String toString(){
    	StringBuilder sb = new StringBuilder();
        sb.append("L(");

		sb.append("origin: " + loiteringOrigin);
		sb.append(", startPoint: " + startPoint);
		sb.append(", startTime: " + startTime);
		sb.append(", speed: " + loiteringSpeed);
		sb.append(", radius: " + loiteringRadius);
		
        sb.append(" )");
        return sb.toString();
    }
	
	
	

}
