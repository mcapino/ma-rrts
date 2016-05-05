package cz.agents.alite.trajectorytools.graph.spatial.region;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public interface SpaceRegion {
	public boolean intersectsLine(SpatialPoint p1, SpatialPoint p2);
	public boolean isInside(SpatialPoint p);	
}
