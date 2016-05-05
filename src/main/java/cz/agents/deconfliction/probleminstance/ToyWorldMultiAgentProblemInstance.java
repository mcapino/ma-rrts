package cz.agents.deconfliction.probleminstance;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public abstract class ToyWorldMultiAgentProblemInstance extends ToyWorldProblemInstance {

    static class CannotPlaceAgentsException extends RuntimeException {};

    double maxTime;

    int nAgents;

    SpatialPoint starts[];
    SpatialPoint targets[];

    Box4dRegion bounds4d;

    public ToyWorldMultiAgentProblemInstance(int cols, int rows, double maxTime, int nAgents, double agentSizeRadius, double obstaclesRatio, int seed) {
        super(cols, rows, obstaclesRatio, agentSizeRadius, seed);
        this.nAgents = nAgents;
        this.maxTime = maxTime;

        starts = new SpatialPoint[nAgents];
        targets = new SpatialPoint[nAgents];

        bounds4d = new Box4dRegion(new TimePoint(bounds3d.getCorner1().x, bounds3d.getCorner1().y, bounds3d.getCorner1().z, 0),
                new TimePoint(bounds3d.getCorner2().x, bounds3d.getCorner2().y, bounds3d.getCorner2().z, maxTime));

        generateMissions();
    }

    public ToyWorldMultiAgentProblemInstance(int cols, int rows, double maxTime, int nAgents, double agentSizeRadius, double obstacledRatio, double obstacleSize, int seed) {
        super(cols, rows, obstacledRatio, obstacleSize, agentSizeRadius, seed);
        this.nAgents = nAgents;
        this.maxTime = maxTime;

        starts = new SpatialPoint[nAgents];
        targets = new SpatialPoint[nAgents];

        bounds4d = new Box4dRegion(new TimePoint(bounds3d.getCorner1().x, bounds3d.getCorner1().y, bounds3d.getCorner1().z, 0),
                new TimePoint(bounds3d.getCorner2().x, bounds3d.getCorner2().y, bounds3d.getCorner2().z, maxTime));

        generateMissions();
    }

    public boolean isUniqueStart(SpatialPoint point) {
        for (int i=0; i<nAgents; i++ ) {
            if (starts[i] != null) {
                if (starts[i].distance(point) < agentSizeRadius*2) {
                    return false;
                }
            }
        }
        return true;
    }

    protected abstract void generateMissions();

    public boolean isUniqueTarget(SpatialPoint point) {
        for (int i=0; i<nAgents; i++ ) {
            if (targets[i] != null) {
                if (targets[i].distance(point) < agentSizeRadius*2) {
                    return false;
                }
            }
        }

        return true;
    }

    public int nAgents() {
        return nAgents;
    }

    public SpatialPoint getStart(int i) {
        return starts[i];
    }

    public SpatialPoint getTarget(int i) {
        return targets[i];
    }

    public Box4dRegion getBounds4d() {
        return bounds4d;
    }

    public SpatialPoint[] getTargets() {
        return targets;
    }

    public SpatialPoint[] getStarts() {
        return starts;
    }


}
