package cz.agents.deconfliction.probleminstance;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ToyWorldSingleAgentProblemInstance extends ToyWorldProblemInstance {

    SpatialPoint start;
    SpatialPoint target;

    public ToyWorldSingleAgentProblemInstance(int cols, int rows, double obstaclesRatio, double agentSizeRadius, int seed) {
        super(cols, rows, obstaclesRatio, agentSizeRadius, seed);
        generateMission();
    }

    private void generateMission() {
        do {
            start = sampleFreeSpace();
            target = sampleFreeSpace();
        } while (start.distance(target) <= 2*getAgentSizeRadius());

    }

    public SpatialPoint getStart() {
        return start;
    }

    public SpatialPoint getTarget() {
        return target;
    }
}
