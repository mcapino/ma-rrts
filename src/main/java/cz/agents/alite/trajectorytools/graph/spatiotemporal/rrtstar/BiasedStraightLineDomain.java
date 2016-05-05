package cz.agents.alite.trajectorytools.graph.spatiotemporal.rrtstar;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.VecUtil;

public class BiasedStraightLineDomain extends SpatioTemporalStraightLineDomain {

    private final double TIME_DEVIATION = (bounds.getCorner2().w - bounds.getCorner1().w) * 0.3;
    private final double ALTITUDE_DEVIATION = (bounds.getCorner2().z - bounds.getCorner1().z) * 0.3;

    Queue<TimePoint> samplesPool = new LinkedList<TimePoint>();

    public BiasedStraightLineDomain(Box4dRegion bounds, TimePoint initialPoint,
            Collection<SpaceTimeRegion> obstacles, SpatialPoint target,
            double targetReachedTolerance, double minSpeed, double optSpeed,
            double maxSpeed, double maxPitch, Random random) {
        super(bounds, initialPoint, obstacles, target, targetReachedTolerance,
                minSpeed, optSpeed, maxSpeed, maxPitch, random);
        samplesPool.add(getTargetSample(initialPoint));
    }


    private TimePoint getTargetSample(TimePoint from) {
        double duration = from.getSpatialPoint().distance(target) / optSpeed;
        return new TimePoint(target, from.getTime() + duration);
    }

    @Override
    public TimePoint sampleState() {

        if (samplesPool.isEmpty()) {
            // generate new random sample
            TimePoint randomSample = super.sampleState();

            // Bias the altidue component and time component
            // of new sample to lie on the straight line
            // between the initialPoint and target


            // bias altitude
            TimePoint interpolatedAltitudeSample = biasAltitude(randomSample, 0);
            TimePoint biasedAltitudeSample = biasAltitude(randomSample, ALTITUDE_DEVIATION);



            // bias time
            TimePoint biasedAltitudeTimeSample = biasTime(biasedAltitudeSample, TIME_DEVIATION);
            samplesPool.offer(biasedAltitudeTimeSample);

            TimePoint interpolatedAltitudeTimeSample = biasTime(interpolatedAltitudeSample, 0);
            samplesPool.offer(interpolatedAltitudeTimeSample);

            // New sample at goal at optimal speed
            samplesPool.offer(getTargetSample(biasedAltitudeTimeSample));
            samplesPool.offer(getTargetSample(interpolatedAltitudeTimeSample));
        }

        return samplesPool.poll();
    }


    private TimePoint biasTime(TimePoint originalPoint, double deviation) {
        double distanceFromInitialPoint = initialPoint.getSpatialPoint().distance(originalPoint.getSpatialPoint());
        double time = initialPoint.getTime() + distanceFromInitialPoint / optSpeed;
        time += random.nextGaussian() * deviation;

        time = MathUtil.clamp(time, bounds.getCorner1().w, bounds.getCorner2().w);

        return new TimePoint(originalPoint.x, originalPoint.y, originalPoint.z, time);
    }


    private TimePoint biasAltitude(TimePoint originalPoint, double deviation) {
        double alpha = 	VecUtil.horizontal(initialPoint.getSpatialPoint()).distance(VecUtil.horizontal(originalPoint.getSpatialPoint())) /
                VecUtil.horizontal(initialPoint.getSpatialPoint()).distance(VecUtil.horizontal(target));
        alpha = MathUtil.clamp(alpha, 0.0, 1.0);

        double z = initialPoint.z + (target.z - initialPoint.z) * alpha;

        z += random.nextGaussian() * deviation;
        z = MathUtil.clamp(z, bounds.getCorner1().z, bounds.getCorner2().z);

        return new TimePoint(originalPoint.x, originalPoint.y, z, originalPoint.w);
    }
}
