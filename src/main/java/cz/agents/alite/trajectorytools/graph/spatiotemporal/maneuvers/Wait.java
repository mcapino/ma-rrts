package cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Vector;

public class Wait extends SpatioTemporalManeuver {
    private static final long serialVersionUID = -2519868162204278196L;

    TimePoint start;
    TimePoint end;

    public Wait(TimePoint start, TimePoint end) {
        super();
        this.start = start;
        this.end = end;
    }

    @Override
    public Trajectory getTrajectory() {

        return new Trajectory() {

            @Override
            public OrientedPoint getPosition(double t) {
                if (t < start.getTime() || t > end.getTime())
                    throw new IllegalArgumentException("The position for time " + t + " which is undefined for this trajectory. Length: " + getDistance() + ". Trajectory defined for interval (" + start.getTime() + ", " + end.getTime() + ")");

                SpatialPoint pos = start.getSpatialPoint();
                Vector dir;
                dir = new Vector(0,1,0);

                return new OrientedPoint(pos, dir);
            }

            @Override
            public double getMinTime() {
                return start.getTime();
            }

            @Override
            public double getMaxTime() {
                return end.getTime();
            }
        };

    }
    @Override
    public double getStartTime() {
        return start.getTime();
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public double getDuration() {
        return end.getTime() - start.getTime();
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((end == null) ? 0 : end.hashCode());
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
        Wait other = (Wait) obj;
        if (end == null) {
            if (other.end != null)
                return false;
        } else if (!end.equals(other.end))
            return false;
        if (start == null) {
            if (other.start != null)
                return false;
        } else if (!start.equals(other.start))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return "[wait: " + start + "=>" + end + "]";
    }

    public TimePoint getStart() {
        return start;
    }

    public TimePoint getEnd() {
        return end;
    }
 }
