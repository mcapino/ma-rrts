package cz.agents.alite.trajectorytools.graph.spatiotemporal.rrtstar;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import javax.vecmath.Vector3d;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.util.OrientedTimePoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class GuidedKinematicStraightLineDomain extends KinematicStraightLineDomain {
	
	Queue<OrientedTimePoint> samplesPool = new LinkedList<OrientedTimePoint>();

    public GuidedKinematicStraightLineDomain(Box4dRegion bounds,
			OrientedTimePoint initialPoint, Collection<SpaceTimeRegion> obstacles,
			SpatialPoint target, double targetReachedTolerance,
			double minSpeed, double optSpeed, double maxSpeed,
			double segmentDistance, double minTurnRadius,
			double maxPitchDeg, Random random) {
		super(bounds, initialPoint, obstacles, target, targetReachedTolerance,
				minSpeed, optSpeed, maxSpeed, segmentDistance, minTurnRadius,
				maxPitchDeg, random);
		samplesPool.add(getTargetSample(initialPoint));
	}




	private OrientedTimePoint getTargetSample(OrientedTimePoint from) {
        double duration = from.getSpatialPoint().distance(target) / optSpeed;
        Vector3d orientation = new Vector3d(target);
        orientation.sub(from.getSpatialPoint());
        orientation.z = 0.0;
        orientation.normalize();
        return new OrientedTimePoint(new TimePoint(target, from.getTime() + duration), orientation);
    }




    @Override
    public OrientedTimePoint sampleState() {
        if (samplesPool.isEmpty()) {
            // generate new random sample
            OrientedTimePoint randomSample = super.sampleState();
            samplesPool.offer(randomSample);

            // Bias towards goal at optimal speed
            samplesPool.offer(getTargetSample(randomSample));

            // Bias towards random sample from initial point at optimal speed
            double optTime = initialPoint.getTime() +
                             initialPoint.getSpatialPoint().distance(randomSample.getSpatialPoint())
                             / optSpeed;

            samplesPool.offer(new OrientedTimePoint(new TimePoint(randomSample.x, randomSample.y, randomSample.z, optTime), randomSample.orientation));
            // Bias towards start altitude
            samplesPool.offer(new OrientedTimePoint(new TimePoint(randomSample.x, randomSample.y, initialPoint.z, randomSample.getTime()), randomSample.orientation));
            // Bias towards start altitude and optimal speed
            samplesPool.offer(new OrientedTimePoint(new TimePoint(randomSample.x, randomSample.y, initialPoint.z, optTime), randomSample.orientation));
        }

        return samplesPool.poll();

    }




	@Override
	public Extension<OrientedTimePoint, SpatioTemporalManeuver> extendTo(
			OrientedTimePoint from, OrientedTimePoint to) {
		Extension<OrientedTimePoint, SpatioTemporalManeuver> extension = super.extendTo(from, to);
		
		return extension;
	}
    
    





}
