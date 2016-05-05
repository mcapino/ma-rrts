package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.Collection;
import java.util.Random;

import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class FlatSpatialStraightLineDomain extends SpatialStraightLineDomain {

    private double defaultZ;
    private SpatialPoint targetPoint;
    private double sampleTargetProbability;
    private Random random = new Random(1);


	public FlatSpatialStraightLineDomain(BoxRegion bounds,
			Collection<SpaceRegion> obstacles, SpaceRegion targetRegion,
			SpatialPoint targetPoint, double speed, double defaultZ,
			double sampleTargetProbability) {
		super(bounds, obstacles, targetRegion, speed);
		this.targetPoint = targetPoint;
		this.defaultZ = defaultZ;
		this.sampleTargetProbability = sampleTargetProbability;
	}

    @Override
    public SpatialPoint sampleState() {
        SpatialPoint point;
        if (random.nextDouble() <= sampleTargetProbability) {
            point = targetPoint;
        } else {
            do {
                double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
                double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
                point = new SpatialPoint(x, y, defaultZ);
            } while (!isInFreeSpace(point));
        }
        return point;
    }

    @Override
    public double estimateCostToGo(SpatialPoint p1) {
        return p1.distance(targetPoint);
    }


}
