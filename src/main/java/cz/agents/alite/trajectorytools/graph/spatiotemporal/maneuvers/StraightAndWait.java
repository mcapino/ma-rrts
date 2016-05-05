package cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Vector;

public class StraightAndWait extends SpatioTemporalManeuver {
    private static final long serialVersionUID = -2519868162204278196L;
    private cz.agents.alite.trajectorytools.graph.spatial.maneuvers.Straight
        spatialStraight;

    TimePoint start;
    TimePoint end;
    double endArrivalTime;
    double speed;

    public StraightAndWait(TimePoint start, TimePoint end, double speed) {
        super();

        this.start = start;
        this.end = end;
        this.speed = speed;
        this.spatialStraight = new cz.agents.alite.trajectorytools.graph.spatial.maneuvers.Straight(start.getSpatialPoint(), end.getSpatialPoint(), speed);
        this.endArrivalTime = start.getTime() + spatialStraight.getDuration();

        // assert that we can make it to the end waypoint before the time specified in end waypoint
        assert((end.getTime() - endArrivalTime)  >= -0.001 );

    }

    @Override
    public Trajectory getTrajectory() {
        return new Trajectory() {

            @Override
            public OrientedPoint getPosition(double t) {
                if (t <= endArrivalTime) {
                    return spatialStraight.getTrajectory(start.getTime()).getPosition(t);
                }
                else if (t > endArrivalTime) {
                    // wait at destination
                    return new OrientedPoint(end.getSpatialPoint(), new Vector(0,1,0));
                } else {
                    throw new RuntimeException("Illegal time given");
                }
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
        return start.getSpatialPoint().distance(end.getSpatialPoint());
    }

    @Override
    public double getDuration() {
        return end.getTime() - start.getTime();
    }

    public double getSpeed() {
        return speed;
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
        StraightAndWait other = (StraightAndWait) obj;
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
        return "[" + start + "=>" + end + " speed:" + speed + " m/s ]";
    }

    public TimePoint getStart() {
        return start;
    }

    public TimePoint getEnd() {
        return end;
    }
 }
