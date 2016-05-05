package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import cz.agents.alite.trajectorytools.util.TimePoint;

/**
 * A region in 4D space
 */
public interface SpaceTimeRegion {
    public boolean intersectsLine(TimePoint p1, TimePoint p2);
    public boolean isInside(TimePoint p);
}
