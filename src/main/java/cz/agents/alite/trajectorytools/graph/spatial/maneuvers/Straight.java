package cz.agents.alite.trajectorytools.graph.spatial.maneuvers;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Vector;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class Straight extends SpatialManeuver {
    private static final long serialVersionUID = -2519868162204278196L;

    private SpatialPoint start;
    private SpatialPoint end;
    private double speed;

    public Straight(Waypoint start, Waypoint end, double speed) {
        super();
        this.start = start;
        this.end = end;
        this.speed = speed;
    }

    public Straight(SpatialPoint start, SpatialPoint end, double speed) {
        super();
        this.start = start;
        this.end = end;
        this.speed = speed;
    }

    @Override
    public Trajectory getTrajectory(final double startTime) {
        return new Trajectory() {

            @Override
            public OrientedPoint getPosition(double t) {
                if (t < startTime || t > startTime + getDuration())
                    throw new IllegalArgumentException("The position for time " + t + " which is undefined for this trajectory. Length: " + getDistance() + ". Trajectory defined for interval (" + startTime + ", " + (startTime + getDuration()) + ")");

                if (getDuration() > 0) {
                    double alpha = (t - startTime) / getDuration();
                    assert(alpha >= -0.00001 && alpha <= 1.00001);

                    SpatialPoint pos = SpatialPoint.interpolate(start, end, alpha);
                    Vector dir;
                    if (!end.equals(start)) {
                        dir = Vector.subtract(end, start);
                        dir.normalize();
                    } else {
                        dir = new Vector(0,1,0);
                    }
                    return new OrientedPoint(pos, dir);
                } else {
                    return new OrientedPoint(start, new Vector(0,1,0));
                }


            }

            @Override
            public double getMinTime() {
                return startTime;
            }

            @Override
            public double getMaxTime() {
                return startTime + getDuration();
            }
        };
    }

    @Override
    public double getDistance() {
        return start.distance(end);
    }

    @Override
    public double getDuration() {
        return getDistance()/speed;
    }

    public double getSpeed() {
        return speed;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((end == null) ? 0 : end.hashCode());
        long temp;
        temp = Double.doubleToLongBits(speed);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        result = prime * result + ((start == null) ? 0 : start.hashCode());
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Straight other = (Straight) obj;
        if (end == null) {
            if (other.end != null)
                return false;
        } else if (!end.equals(other.end))
            return false;
        if (Double.doubleToLongBits(speed) != Double
                .doubleToLongBits(other.speed))
            return false;
        if (start == null) {
            if (other.start != null)
                return false;
        } else if (!start.equals(other.start))
            return false;
        return true;
    }



}
