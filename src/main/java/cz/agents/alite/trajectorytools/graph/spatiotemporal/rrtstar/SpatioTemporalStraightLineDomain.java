package cz.agents.alite.trajectorytools.graph.spatiotemporal.rrtstar;

import java.util.Collection;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.log4j.Logger;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.Straight;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.Wait;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class SpatioTemporalStraightLineDomain implements Domain<TimePoint, SpatioTemporalManeuver> {

    private static final Logger LOGGER = Logger.getLogger(SpatioTemporalStraightLineDomain.class);

    private static final double NONOPTIMAL_SPEED_PENALTY_COEF = 5.0; /*0.5 1.5 5.0*/
    private static final double CLIMB_PENALTY_COEF = 1.0;

    Box4dRegion bounds;
    Collection<SpaceTimeRegion> obstacles;

    TimePoint initialPoint;
    SpatialPoint target;
    double targetReachedTolerance;

    double minSpeed;
    double optSpeed;
    double maxSpeed;

    double maxPitch;

    Random random;

    public SpatioTemporalStraightLineDomain(Box4dRegion bounds, TimePoint initialPoint,
            Collection<SpaceTimeRegion> obstacles, SpatialPoint target, double targetReachedTolerance, double minSpeed,
            double optSpeed, double maxSpeed, double maxPitch, Random random) {
        super();

        this.bounds = bounds;
        this.initialPoint = initialPoint;
        this.obstacles = obstacles;
        this.target = target;
        this.targetReachedTolerance = targetReachedTolerance;
        this.minSpeed = minSpeed;
        this.optSpeed = optSpeed;
        this.maxSpeed = maxSpeed;
        this.maxPitch = maxPitch;
        this.random = random;

        if (!isInFreeSpace(initialPoint)) {
            throw new IllegalArgumentException("Initial point " + initialPoint + "is not in free space");
        }
    }

    @Override
    public TimePoint sampleState() {
        TimePoint point;
        do {
            double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
            double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
            double z = bounds.getCorner1().z + (random.nextDouble() * (bounds.getCorner2().z - bounds.getCorner1().z));
            double t = bounds.getCorner1().w + (random.nextDouble() * (bounds.getCorner2().w - bounds.getCorner1().w));
            point = new TimePoint(x, y, z, t);
        } while (!isInFreeSpace(point));
        return point;
    }

    public TimePoint sampleState(double t) {
        TimePoint point;
        do {
            double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
            double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
            double z = bounds.getCorner1().z + (random.nextDouble() * (bounds.getCorner2().z - bounds.getCorner1().z));
            point = new TimePoint(x, y, z, t);
        } while (!isInFreeSpace(point));
        return point;
    }

    @Override
    public Extension<TimePoint, SpatioTemporalManeuver> extendTo(
            TimePoint from, TimePoint to) {
        return extendMaintainSpatialPoint(from, to);
   }

    public Extension<TimePoint, SpatioTemporalManeuver> steerMaintainSpatialPoint(
            TimePoint from, TimePoint to) {

        double distance = from.getSpatialPoint().distance(to.getSpatialPoint());
        double requiredSpeed = distance / (to.getTime() - from.getTime());
        boolean exact = false;
        double actualSpeed = MathUtil.clamp(requiredSpeed, minSpeed, maxSpeed);
        TimePoint extensionTarget = new TimePoint(to.getSpatialPoint(), from.getTime() + distance / actualSpeed);
        SpatioTemporalManeuver maneuver = new Straight(from, extensionTarget);
        double cost = evaluateFuelCost(from.getSpatialPoint(), extensionTarget.getSpatialPoint(), actualSpeed);

        if (extensionTarget.epsilonEquals(to, 0.001)) {
            extensionTarget = to;
            exact = true;
        }

        if (satisfiesDynamicLimits(from, extensionTarget)) {
            return new Extension<TimePoint, SpatioTemporalManeuver>(from, extensionTarget,
                       maneuver, cost, exact);
        } else {
            return null;
        }
   }


    public Extension<TimePoint, SpatioTemporalManeuver> extendMaintainSpatialPoint(
            TimePoint from, TimePoint to) {

        double distance = from.getSpatialPoint().distance(to.getSpatialPoint());
        double requiredSpeed = distance / (to.getTime() - from.getTime());
        boolean exact = false;
        double actualSpeed = MathUtil.clamp(requiredSpeed, minSpeed, maxSpeed);
        TimePoint extensionTarget = new TimePoint(to.getSpatialPoint(), from.getTime() + distance / actualSpeed);
        SpatioTemporalManeuver maneuver = new Straight(from, extensionTarget);
        double cost = evaluateFuelCost(from.getSpatialPoint(), extensionTarget.getSpatialPoint(), actualSpeed);

        if (extensionTarget.epsilonEquals(to, 0.001)) {
            extensionTarget = to;
            exact = true;
        }

        if (satisfiesDynamicLimits(from, extensionTarget) &&
            !intersectsObstacles(from, extensionTarget)) {
            return new Extension<TimePoint, SpatioTemporalManeuver>(from, extensionTarget,
                       maneuver, cost, exact);
        } else {
            return null;
        }
   }



    public Extension<TimePoint, SpatioTemporalManeuver> extendMaintainTime(
            TimePoint from, TimePoint to) {

        double distance = from.getSpatialPoint().distance(to.getSpatialPoint());
        double requiredDuration = to.getTime() - from.getTime();
        double speed = distance / requiredDuration;

        boolean exact = (speed >= minSpeed && speed <= maxSpeed);

        if (from.getSpatialPoint().epsilonEquals(to.getSpatialPoint(), 0.001) && isInTargetRegion(from)) {
            // Wait at target
            return new Extension<TimePoint, SpatioTemporalManeuver>(from, to, new Wait(from, to), 0, true);
        }

        TimePoint extensionTarget;
        if (exact) {
            extensionTarget = to;
        } else {
            Vector3d dir = new Vector3d(to.getSpatialPoint());
            dir.sub(from.getSpatialPoint());
            dir.normalize();
            dir.scale(requiredDuration * optSpeed);
            Point3d spatialTarget = new Point3d(from.getSpatialPoint());
            spatialTarget.add(dir);
            extensionTarget = new TimePoint(spatialTarget, to.getTime());
        }

        if (bounds.isInside(extensionTarget)) {
            SpatioTemporalManeuver maneuver = new Straight(from, extensionTarget);
            speed = from.getSpatialPoint().distance(extensionTarget.getSpatialPoint());
            double cost = evaluateFuelCost(from.getSpatialPoint(), extensionTarget.getSpatialPoint(), speed);

            if (satisfiesDynamicLimits(from, extensionTarget) &&
                !intersectsObstacles(from, extensionTarget)) {
                return new Extension<TimePoint, SpatioTemporalManeuver>(from, extensionTarget, maneuver, cost, exact);
            }
        }

        return null;
   }


    @Override
    public ExtensionEstimate estimateExtension(
            TimePoint from, TimePoint to) {

        double distance = from.getSpatialPoint().distance(to.getSpatialPoint());
        double requiredSpeed = distance / (to.getTime() - from.getTime());
        boolean exact = (requiredSpeed >= minSpeed && requiredSpeed <= maxSpeed);
        double actualSpeed = MathUtil.clamp(requiredSpeed, minSpeed, maxSpeed);
        double cost = evaluateFuelCost(from.getSpatialPoint(), to.getSpatialPoint(), actualSpeed);

        return new ExtensionEstimate(cost, exact);
    }

    @Override
    public double estimateCostToGo(TimePoint p) {
        return evaluateFuelCost(p.getSpatialPoint(), target, optSpeed);
    }

    @Override
    public double distance(TimePoint p1, TimePoint p2) {

        //return p1.distance(p2);

        double speed = p1.getSpatialPoint().distance(p2.getSpatialPoint()) / Math.abs(p2.getTime() - p1.getTime());

        if (speed >= minSpeed && speed <= maxSpeed) {
            return evaluateFuelCost(p1.getSpatialPoint(), p2.getSpatialPoint(), speed);
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    @Override
    public double nDimensions() {
        return 4;
    }

    @Override
    public boolean isInTargetRegion(TimePoint p) {
        return target.distance(p.getSpatialPoint()) <= targetReachedTolerance;
    }

    protected boolean isInFreeSpace(TimePoint p) {
        if (bounds.isInside(p)) {
            for (SpaceTimeRegion obstacle : obstacles) {
                if (obstacle.isInside(p)) {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    // Simple approximation that should force the planner to use optimal cruise speed
    protected double evaluateFuelCost(SpatialPoint start, SpatialPoint end, double speed) {
        double cost =  start.distance(end) * ((Math.abs(speed - optSpeed) / optSpeed) * NONOPTIMAL_SPEED_PENALTY_COEF + 1.0);
        double climbPenalty = MathUtil.clamp(end.z - start.z, 0, Double.POSITIVE_INFINITY) * CLIMB_PENALTY_COEF;
        return cost + climbPenalty;
    }

    protected boolean intersectsObstacles(TimePoint p1, TimePoint p2) {
            // check obstacles
            for (SpaceTimeRegion obstacle : obstacles) {
                if (obstacle.intersectsLine(p1, p2)) {
                    return true;
                }
            }
            return false;
    }

    protected boolean satisfiesDynamicLimits(TimePoint p1, TimePoint p2) {
        // check speed constraints
        double requiredSpeed = (p1.getSpatialPoint().distance(p2.getSpatialPoint()))
                / (p2.getTime() - p1.getTime());

        // check pitch limit
        double horizontalDistance = (new Point2d(p1.x, p1.y)).distance(new Point2d(p2.x, p2.y));
        double verticalDistance = (p2.z-p1.z);
        double climbAngleDeg = Math.atan(verticalDistance/horizontalDistance) * 180/Math.PI;

        if (Math.abs(climbAngleDeg) > maxPitch) {
            return false;
        }

        if (!(requiredSpeed >= minSpeed - 0.001 || requiredSpeed <= maxSpeed + 0.001)) {
            return false;
        }

        return true;
    }

}
