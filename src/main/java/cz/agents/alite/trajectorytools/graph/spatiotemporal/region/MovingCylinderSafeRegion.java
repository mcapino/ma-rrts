package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import javax.vecmath.Point2d;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class MovingCylinderSafeRegion implements SpaceTimeRegion {
    double samplingInterval;
    Trajectory trajectory;
    private double halfHeight;
    double radius;


    public MovingCylinderSafeRegion(Trajectory trajectory, double radius, double halfHeight, double samplingInterval) {
        super();
        this.trajectory = trajectory;
        this.radius = radius;
        this.halfHeight = halfHeight;
        this.samplingInterval = samplingInterval;
    }

    @Override
    public boolean intersectsLine(TimePoint p1, TimePoint p2) {
        TimePoint start;
        TimePoint end;

        if (p1.getTime() < p2.getTime()) {
            start = p1;
            end = p2;
        } else {
            start = p2;
            end = p1;
        }

        double tmin = Math.max(trajectory.getMinTime(), start.getTime());
        double tmax = Math.min(trajectory.getMaxTime(), end.getTime());

        for (double t=tmin; t <= tmax; t += samplingInterval) {
            double alpha = (t - start.getTime())
                    / (end.getTime() - start.getTime());
            assert (alpha >= -0.00001 && alpha <= 1.00001);
            SpatialPoint linePos = SpatialPoint.interpolate(
                    start.getSpatialPoint(), end.getSpatialPoint(), alpha);
            SpatialPoint trajPos = trajectory.getPosition(t);
            
            if ((new Point2d(trajPos.x,trajPos.y)).distance(new Point2d(linePos.x, linePos.y)) <= radius && 
            	Math.abs(trajPos.z - linePos.z) < halfHeight) {
                return true;
            }
        }

        return false;

    }

    @Override
    public boolean isInside(TimePoint p) {
        if (p.getTime() >= trajectory.getMinTime() && p.getTime() <= trajectory.getMaxTime()) {
        	SpatialPoint trajPos = trajectory.getPosition(p.getTime());
        	if ((new Point2d(trajPos.x,trajPos.y)).distance(new Point2d(p.x, p.y)) <= radius && 
                	Math.abs(trajPos.z - p.z) <= halfHeight) {
                    return true;
                } else {
                	return false;
                }
        } else {
            return false;
        }

    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public double getRadius() {
        return radius;
    }

}
