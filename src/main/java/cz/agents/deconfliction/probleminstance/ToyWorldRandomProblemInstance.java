package cz.agents.deconfliction.probleminstance;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ToyWorldRandomProblemInstance extends
        ToyWorldMultiAgentProblemInstance {

    public ToyWorldRandomProblemInstance(int cols, int rows,
            double maxTime, int nAgents, double agentSizeRadius, double obstaclesRatio, int seed) {
        super(cols, rows, maxTime, nAgents, agentSizeRadius, obstaclesRatio, seed);
    }

    public ToyWorldRandomProblemInstance(int cols, int rows,
            double maxTime, int nAgents, double agentSizeRadius, double obstacledRatio, double obstacleSize, int seed) {
        super(cols, rows, maxTime, nAgents, agentSizeRadius, obstacledRatio, obstacleSize, seed);
    }

    @Override
    protected void generateMissions() {
        int trials = 0;
        for (int i=0; i<nAgents; i++) {
            SpatialPoint start;
            SpatialPoint target;

            do {
                start  = sampleFreeSpace();
                target = sampleFreeSpace();

                trials++;
                if (trials > 10000) {
                    throw new CannotPlaceAgentsException();
                }
            }
            while (!isUniqueStart(start) || !isUniqueTarget(target));

            starts[i] = start;
            targets[i] = target;
        }
    }



}
