package cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Vector;

public class Straight extends SpatioTemporalManeuver {
    private static final long serialVersionUID = -2519868162204278196L;
    private cz.agents.alite.trajectorytools.graph.spatial.maneuvers.Straight
        spatialStraight;

    TimePoint start;
    TimePoint end;

    public Straight(TimePoint start, TimePoint end) {
        super();
        this.start = start;
        this.end = end;
        double speed = start.getSpatialPoint().distance(end.getSpatialPoint()) / (end.getTime() - start.getTime());
        this.spatialStraight = new cz.agents.alite.trajectorytools.graph.spatial.maneuvers.Straight(start.getSpatialPoint(), end.getSpatialPoint(), speed);
    }

    @Override
    public Trajectory getTrajectory() {
        return spatialStraight.getTrajectory(start.getTime());
    }

    @Override
    public double getStartTime() {
        return start.getTime();
    }

    @Override
    public double getDistance() {
        return start.getSpatialPoint().distance(end.getSpatialPoint());
    }

    @Override
    public double getDuration() {
        return end.getTime() - start.getTime();
    }

    public double getSpeed() {
        return getDistance()/getDuration();
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
        Straight other = (Straight) obj;
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
        return "[" + start + "=>" + end + " spped:" + spatialStraight.getSpeed() + " m/s ]";
    }

    public TimePoint getStart() {
        return start;
    }

    public TimePoint getEnd() {
        return end;
    }
 }
