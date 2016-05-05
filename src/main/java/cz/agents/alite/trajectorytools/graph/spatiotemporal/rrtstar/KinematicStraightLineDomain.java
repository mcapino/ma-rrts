package cz.agents.alite.trajectorytools.graph.spatiotemporal.rrtstar;

import java.util.Collection;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import cz.agents.alite.planner.spatialmaneuver.PathFindSpecification;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ExpandManeuver;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ExpandManeuverType;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ExpandManeuvers;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ManeuverSpecification;
import cz.agents.alite.planner.spatialmaneuver.maneuver.ManeuverSpecification.LevelConstants;
import cz.agents.alite.planner.spatialmaneuver.maneuver.SmoothManeuver;
import cz.agents.alite.planner.spatialmaneuver.zone.EmptyZone;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.Straight;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.OrientedTimePoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.VecUtil;
import cz.agents.alite.trajectorytools.util.Vector;

public class KinematicStraightLineDomain implements Domain<OrientedTimePoint, SpatioTemporalManeuver> {

    private static final double NONOPTIMAL_SPEED_PENALTY_COEF = 0.1;

    Box4dRegion bounds;
    Collection<SpaceTimeRegion> obstacles;

    OrientedTimePoint initialPoint;
    SpatialPoint target;
    double targetReachedTolerance;

    double minSpeed;
    double optSpeed;
    double maxSpeed;

    double segmentDistance;
    double maxAbsPitchRad;
    double minTurnRadius;


    Random random;

    public KinematicStraightLineDomain(Box4dRegion bounds, OrientedTimePoint initialPoint,
            Collection<SpaceTimeRegion> obstacles, SpatialPoint target, double targetReachedTolerance, double minSpeed,
            double optSpeed, double maxSpeed, double minSegmentDistance, double minTurnRadius, double maxPitchDeg, Random random) {
        super();

        this.bounds = bounds;
        this.initialPoint = initialPoint;
        this.obstacles = obstacles;
        this.target = target;
        this.targetReachedTolerance = targetReachedTolerance;
        this.minSpeed = minSpeed;
        this.optSpeed = optSpeed;
        this.maxSpeed = maxSpeed;
        this.maxAbsPitchRad = maxPitchDeg * Math.PI/180.0;
        this.segmentDistance = minSegmentDistance;
        this.minTurnRadius = minTurnRadius;
        this.random = random;

        if (!isInFreeSpace(initialPoint)) {
            throw new IllegalArgumentException("Initial point is not in free space");
        }
    }

    @Override
    public OrientedTimePoint sampleState() {
        OrientedTimePoint point;
        do {
            double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
            double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
            double z = bounds.getCorner1().z + (random.nextDouble() * (bounds.getCorner2().z - bounds.getCorner1().z));
            double t = bounds.getCorner1().w + (random.nextDouble() * (bounds.getCorner2().w - bounds.getCorner1().w));
            // angle
            double dx = random.nextDouble()*2 - 1;
            double dy = random.nextDouble()*2 - 1;
            double dz = 0;
            point = new OrientedTimePoint(new TimePoint(x, y, z, t), new Vector(dx, dy, dz) );
        } while (!isInFreeSpace(point));
        return point;
    }



   public Extension<OrientedTimePoint, SpatioTemporalManeuver> steer(
            OrientedTimePoint from, OrientedTimePoint to) {

        double requiredDistance = from.getSpatialPoint().distance(to.getSpatialPoint());

        if (requiredDistance < 0.001) return null;

        // Compute required speed
        double requiredSpeed = requiredDistance / (to.getTime() - from.getTime());

        // Compute yaw step needed to reach "to" point spatially
        Vector2d toVector = new Vector2d(VecUtil.horizontal(to.getSpatialPoint()));
        toVector.sub(VecUtil.horizontal(from.getSpatialPoint()));
        double requiredYawChangeToReachTargetPoint = VecUtil.angle(VecUtil.horizontal(from.orientation), toVector);

        // Compute yaw step needed to reach "to" orientation
        double requiredYawChangeToReachTargetOrientation  = VecUtil.angle(VecUtil.horizontal(from.orientation), VecUtil.horizontal(to.orientation));

        // Compute pitch step needed
        double verticalDistance = to.z - from.z;
        double horizontalDistance = VecUtil.horizontal(from.getSpatialPoint()).distance(VecUtil.horizontal(to.getSpatialPoint()));
        double requiredPitchAngleInRad = horizontalDistance != 0 ? Math.atan(verticalDistance / horizontalDistance) : 0;

        double maxYawChangeInSegment = Math.min(requiredDistance / (2 * minTurnRadius),  segmentDistance / (2 * minTurnRadius));

        double distance;
        if (Math.abs(requiredYawChangeToReachTargetPoint) <= maxYawChangeInSegment  &&
            Math.abs(requiredPitchAngleInRad) <= maxAbsPitchRad &&
            requiredSpeed >= minSpeed && requiredSpeed <= maxSpeed
            ) {
            // spatially exact segment is possible
            distance = requiredDistance;
        } else {
            // spatially exact segment is not possible
            distance = Math.min(requiredDistance, segmentDistance);
            maxYawChangeInSegment = distance / (2 * minTurnRadius);
        }

        // Clamp to kinematic limits
        double speed = MathUtil.clamp(requiredSpeed, minSpeed, maxSpeed);
        double yawChangeToTargetPointInRad = MathUtil.clamp(requiredYawChangeToReachTargetPoint, -maxYawChangeInSegment, maxYawChangeInSegment);
        double yawChangeToTargetOrientationInRad = MathUtil.clamp(requiredYawChangeToReachTargetOrientation, -maxYawChangeInSegment, maxYawChangeInSegment);
        double pitchRad = MathUtil.clamp(requiredPitchAngleInRad, -maxAbsPitchRad, maxAbsPitchRad);

        // Compute the target point
        Vector2d horizontalOrientation = VecUtil.horizontal(from.orientation);
        horizontalOrientation = VecUtil.rotateYaw(horizontalOrientation, yawChangeToTargetPointInRad);
        horizontalOrientation.normalize();

        Vector3d edgeVector = new Vector3d(horizontalOrientation.x, horizontalOrientation.y, Math.tan(pitchRad));
        edgeVector.normalize();
        edgeVector.scale(distance);
        Point3d targetSpatialPoint = from.getSpatialPoint();
        targetSpatialPoint.add(edgeVector);

        // compute target orientation
        Vector2d fromHorizontalOrientation = VecUtil.horizontal(from.orientation);
        fromHorizontalOrientation = VecUtil.rotateYaw(fromHorizontalOrientation, yawChangeToTargetOrientationInRad);
        fromHorizontalOrientation.normalize();

        double targetOrientationPitchRad = Math.asin(to.orientation.z);
        Vector3d targetOrientation = new Vector3d(horizontalOrientation.x, horizontalOrientation.y, Math.tan(targetOrientationPitchRad));
        targetOrientation.normalize();

        // compute target time
        double targetTime = from.getTime() + distance / speed;

        OrientedTimePoint target = new OrientedTimePoint(new TimePoint(targetSpatialPoint, targetTime), targetOrientation);
        SpatioTemporalManeuver maneuver = new Straight(from, target);
        double cost = evaluateFuelCost(from.getSpatialPoint(), target.getSpatialPoint(), speed);

        boolean exact;
        if (target.epsilonEquals(to, 0.0001)) {
            target = to;
            exact = true;
        } else {
            exact = false;
        }

        assert(satisfiesDynamicLimits(from, target));

        return new Extension<OrientedTimePoint, SpatioTemporalManeuver>(from, target, maneuver, cost, exact);
   }

   @Override
   public Extension<OrientedTimePoint, SpatioTemporalManeuver> extendTo(
           OrientedTimePoint from, OrientedTimePoint to) {
       return extendMaintainSpatialPoint(from, to);
  }

   public Extension<OrientedTimePoint, SpatioTemporalManeuver> extendMaintainSpatialPoint(
            OrientedTimePoint from, OrientedTimePoint to) {

        Extension<OrientedTimePoint, SpatioTemporalManeuver> extension = steer(from, to);

        if (extension != null && !intersectsObstacles(extension.source, extension.target)) {
            return extension;
        } else {
            return null;
        }
   }


    @Override
    public ExtensionEstimate estimateExtension(OrientedTimePoint from, OrientedTimePoint to) {

        Extension<OrientedTimePoint, SpatioTemporalManeuver> extension = steer(from, to);
        if (extension != null) {
            return new ExtensionEstimate(extension.cost, extension.exact);
        } else {
            return null;
        }
    }

    @Override
    public double estimateCostToGo(OrientedTimePoint p) {
        return evaluateFuelCost(p.getSpatialPoint(), target, optSpeed);
    }

    @Override
    public double distance(OrientedTimePoint from, OrientedTimePoint to) {
        return dubins3dDistance(from, to, optSpeed);
        //return p1.distance(p2);
    }

    @Override
    public double nDimensions() {
        return 7;
    }

    @Override
    public boolean isInTargetRegion(OrientedTimePoint p) {
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
        return start.distance(end) * ((Math.abs(speed - optSpeed) / optSpeed) * NONOPTIMAL_SPEED_PENALTY_COEF + 1.0);
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
        double climbAngleDeg = Math.atan(verticalDistance/horizontalDistance);

        if (Math.abs(climbAngleDeg) - maxAbsPitchRad > 0.01) {
            return false;
        }

        if (!(requiredSpeed >= minSpeed - 0.001 || requiredSpeed <= maxSpeed + 0.001)) {
            return false;
        }

        return true;
    }


    public double dubins3dDistance(OrientedTimePoint from, OrientedTimePoint to, double speed) {

        double mainSegmentDistance = dubins3dDistance(from.getOrientedPoint(), to.getOrientedPoint());
        double mainSegmentDuration = mainSegmentDistance/speed;

        if ((to.getTime() - from.getTime()) - mainSegmentDuration > 0) {
            double loopDistance = ((to.getTime() - from.getTime()) - mainSegmentDuration) * speed;
            if (loopDistance >= 2*Math.PI*minTurnRadius) {
                return mainSegmentDistance + loopDistance;
            }
        } else {
            // The main segment cannot be constructed in the available time
        }
        return Double.POSITIVE_INFINITY;

    }

    public double dubins3dDistance(OrientedPoint from, OrientedPoint to) {

        // plan trajectory constants
        LevelConstants[] levelConstants =  new LevelConstants[2];
        levelConstants[0] = new LevelConstants(2, 12, 8);

        ExpandManeuvers expandManeuvers = new ExpandManeuvers();
        expandManeuvers.getManeuver().add(new ExpandManeuver(ExpandManeuverType.STRAIGHT));

        ManeuverSpecification maneuverSpecification = new ManeuverSpecification(null, levelConstants, expandManeuvers);
        PathFindSpecification specification = new PathFindSpecification(minTurnRadius, new EmptyZone(), maneuverSpecification );
        SmoothManeuver smooth = new SmoothManeuver(from, from.orientation, 0, to, to.orientation, specification);

        return smooth.getLength();
    }

}
