package cz.agents.alite.trajectorytools.graph.jointtoyworld.rrtstar;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Random;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalState;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.StraightAndWait;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.Extension;
import cz.agents.alite.trajectorytools.planner.rrtstar.ExtensionEstimate;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.deconfliction.probleminstance.ToyWorldMultiAgentProblemInstance;

public class JointToyWorldDomain implements Domain<JointSpatioTemporalState, JointSpatioTemporalManeuver> {

    //SpatialStraightLineDomain[] domains;
    protected double separation;
    protected Box4dRegion bounds;
    protected Random random;
    protected double samplingInterval;
    protected double speed = 1.0;
    protected SpatialPoint[] targets;
    protected double targetReachedTolerance;
    protected ToyWorldMultiAgentProblemInstance problem;


    public JointToyWorldDomain(ToyWorldMultiAgentProblemInstance problem, double speed, Random random) {
        super();

        this.separation = problem.getAgentSizeRadius() * 2;
        this.bounds = problem.getBounds4d();
        this.random = random;
        this.samplingInterval = (separation / 4) / speed;
        this.targets = problem.getTargets();
        this.targetReachedTolerance = problem.getCellSize() / 2;
        this.problem = problem;
    }

    @Override
    public JointSpatioTemporalState sampleState() {
        TimePoint[] points = new TimePoint[nAgents()];
        do {
            // pick time
            double t = bounds.getCorner1().w + random.nextDouble() * (bounds.getCorner2().w - bounds.getCorner1().w);

            // pick spatial point for each agent
            for (int i = 0; i < nAgents(); i++) {

                points[i] = new TimePoint(problem.sampleFreeSpace(),t);
            }
        } while (!separated(points, separation)); /* not necessary probably, will be checked when extending anyway */

        return new JointSpatioTemporalState(points);
    }

    private Point3d getRandomSpatialPoint() {
        double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
        double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
        double z = 0;
        return new SpatialPoint(x, y, z);
    }

    @Override
    public Extension<JointSpatioTemporalState, JointSpatioTemporalManeuver> extendTo(JointSpatioTemporalState from, JointSpatioTemporalState to) {

        if (reachable(from, to)) {
            SpatioTemporalManeuver[] maneuvers = new SpatioTemporalManeuver[from.nAgents()];

            for (int i = 0; i < from.nAgents(); i++) {
                maneuvers[i] = new StraightAndWait(from.get(i), to.get(i), speed);
            }

            // Check separation between individual trajectories

            Collection<Trajectory> trajectories = new LinkedList<Trajectory>();
            for(int i = 0; i < maneuvers.length; i++) {
                trajectories.add(maneuvers[i].getTrajectory());
            }

            if (!SeparationDetector.hasConflict(trajectories, separation, samplingInterval)) {
                return new Extension<JointSpatioTemporalState, JointSpatioTemporalManeuver> (from, to, new JointSpatioTemporalManeuver(maneuvers), evaluateCost(maneuvers), true);
            }
        }
        return null;

    }

    private double evaluateCost(SpatioTemporalManeuver[] maneuvers) {
        double cost = 0.0;
        for (int i = 0; i < maneuvers.length; i++) {
            cost += maneuvers[i].getDuration();
        }
        return cost;
    }

    @Override
    public ExtensionEstimate estimateExtension(JointSpatioTemporalState from, JointSpatioTemporalState to) {

        if (reachable(from, to)) {
            SpatioTemporalManeuver[] maneuvers = new SpatioTemporalManeuver[from.nAgents()];

            for (int i = 0; i < from.nAgents(); i++) {
                maneuvers[i] = new StraightAndWait(from.get(i), to.get(i), speed);
            }
            return new ExtensionEstimate(evaluateCost(maneuvers), true);
        } else {
            return null;
        }
    }

    private boolean reachable(JointSpatioTemporalState from, JointSpatioTemporalState to) {
        double timeDiff = to.getTime() - from.getTime();
        double distance = from.maxSpatialDistance(to);

        return ((distance/speed) <= timeDiff);
    }

    @Override
    public double estimateCostToGo(JointSpatioTemporalState s) {
        double lastAtGoalTime = 0.0;

        // assuming no conflicts and obstacles...
        // determine when the last agents arrives to the goal
        for (int i=0; i < nAgents(); i++) {
            double atGoalTime = s.get(i).getTime() + s.get(i).getSpatialPoint().distance(targets[i]) / speed;
            if (lastAtGoalTime < atGoalTime) {
                lastAtGoalTime = atGoalTime;
            }
        }

        // build a joint maneuver to the target
        SpatioTemporalManeuver[] maneuvers = new SpatioTemporalManeuver[nAgents()];
        for (int i=0; i < nAgents(); i++) {
            maneuvers[i] = new StraightAndWait(s.get(i), new TimePoint(targets[i], lastAtGoalTime), speed);
        }

        return evaluateCost(maneuvers);
    }

    @Override
    public double distance(JointSpatioTemporalState s1, JointSpatioTemporalState s2) {
        if (reachable(s1, s2)) {
            return s1.sumOfSpatialDistances(s2);
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    @Override
    public double nDimensions() {
        return (2*nAgents()) + 1;
    }

    @Override
    public boolean isInTargetRegion(JointSpatioTemporalState s) {
        for (int i = 0; i < s.nAgents(); i++) {
           if (s.get(i).getSpatialPoint().distance(targets[i]) > targetReachedTolerance ) {
               return false;
           }
        }
        return true;
    }

    public int nAgents() {
        return targets.length;
    }

    private static boolean separated(TimePoint[] timePoints, double separation) {
        for (int i=0; i<timePoints.length; i++) {
            for (int j=i+1; j<timePoints.length; j++) {
                if (Math.abs(timePoints[i].getTime() - timePoints[j].getTime()) < 0.001) {
                    if (timePoints[i].getSpatialPoint().distance(timePoints[j].getSpatialPoint()) < separation) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

}
