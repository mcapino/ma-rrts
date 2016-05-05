package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class MovingSphereSafeRegion implements SpaceTimeRegion {
    double samplingInterval;
    Trajectory trajectory;
    double radius;

    public MovingSphereSafeRegion(Trajectory trajectory, double radius, double samplingInterval) {
        super();
        this.trajectory = trajectory;
        this.radius = radius;
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
            SpatialPoint pos = SpatialPoint.interpolate(
                    start.getSpatialPoint(), end.getSpatialPoint(), alpha);

            if (trajectory.getPosition(t).distance(pos) <= radius) {
                return true;
            }
        }

        return false;

    }

    @Override
    public boolean isInside(TimePoint p) {
        if (p.getTime() >= trajectory.getMinTime() && p.getTime() <= trajectory.getMaxTime()) {
            return (p.getSpatialPoint()
                .distance(trajectory.getPosition(p.getTime())) <= radius);
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
