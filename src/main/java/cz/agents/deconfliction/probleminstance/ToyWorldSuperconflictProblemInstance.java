package cz.agents.deconfliction.probleminstance;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ToyWorldSuperconflictProblemInstance extends
        ToyWorldMultiAgentProblemInstance {

    public ToyWorldSuperconflictProblemInstance(int cols, int rows,
            double maxTime, int nAgents, double agentSizeRadius, double obstaclesRatio, int seed) {
        super(cols, rows, maxTime, nAgents, agentSizeRadius, obstaclesRatio, seed);
    }

    public ToyWorldSuperconflictProblemInstance(int cols, int rows,
            double maxTime, int nAgents, double agentSizeRadius, double obstacledRatio, double obstacleSize, int seed) {
        super(cols, rows, maxTime, nAgents, agentSizeRadius, obstacledRatio, obstacleSize, seed);
    }

    @Override
    protected void generateMissions() {

        SpatialPoint center = bounds4d.getCenter().getSpatialPoint();
        double circleRadius = (bounds4d.getXSize()/2.0 * 0.7);

        for (int i = 0; i < nAgents; i++ ) {
            double angle = i*(2*Math.PI/nAgents);
            starts[i] = new SpatialPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius, center.z);
            targets[i] = new SpatialPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius, center.z);
        }
    }



}
