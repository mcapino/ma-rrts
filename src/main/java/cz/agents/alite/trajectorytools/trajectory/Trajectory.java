package cz.agents.alite.trajectorytools.trajectory;

import cz.agents.alite.trajectorytools.util.OrientedPoint;

/**
 * Trajectory represents spatial positions and orientations of an object as a
 * function of time. The spatial position and orientation of an object in given
 * time can be obtained using {@link #getPosition(double)} and must be defined
 * for all time points s.t. {@link #getMinTime()} <= time <=
 * {@link #getMaxTime()}. For all other times, {@link #getPosition(double)}
 * returns null.
 */
public interface Trajectory {
    public double getMinTime();
    public double getMaxTime();
    public OrientedPoint getPosition(double t);
}
