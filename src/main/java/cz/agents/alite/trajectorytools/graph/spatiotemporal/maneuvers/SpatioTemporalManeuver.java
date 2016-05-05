package cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers;

import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public abstract class SpatioTemporalManeuver extends DefaultWeightedEdge {
    private static final long serialVersionUID = -7391923415142790579L;

    /**
     * @param startTime time at which the maneuver starts
     */
    abstract public Trajectory getTrajectory();

    abstract public double getStartTime();

    /**
     * @return the length of the maneuver in meters
     */
    abstract public double getDistance();

    /**
     * @return the duration of the maneuver in seconds
     */
    abstract public double getDuration();
}
