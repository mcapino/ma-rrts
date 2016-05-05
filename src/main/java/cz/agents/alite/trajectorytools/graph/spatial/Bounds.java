package cz.agents.alite.trajectorytools.graph.spatial;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class Bounds {
		final public double minx;
		final public double maxx;
		
		final public double miny;
		final public double maxy;
		
		final public double minz;
		final public double maxz;	

		
		public Bounds(double minx, double maxx, double miny,
				double maxy, double minz, double maxz) {
			super();
			this.minx = minx;
			this.maxx = maxx;
			this.miny = miny;
			this.maxy = maxy;
			this.minz = minz;
			this.maxz = maxz;
		}	
		
		public boolean isInside(SpatialPoint point) {
			return minx <= point.x &&
					maxx >= point.x &&
					miny <= point.y &&
					maxy >= point.y &&
					minz <= point.z &&
					maxz >= point.z ;
		}
}
