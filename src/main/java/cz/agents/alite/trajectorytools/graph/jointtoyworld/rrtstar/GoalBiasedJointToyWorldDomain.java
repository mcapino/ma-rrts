package cz.agents.alite.trajectorytools.graph.jointtoyworld.rrtstar;

import java.util.Random;

import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalState;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.deconfliction.probleminstance.ToyWorldMultiAgentProblemInstance;

public class GoalBiasedJointToyWorldDomain extends JointToyWorldDomain {

    protected double strength = 0.2;

    public GoalBiasedJointToyWorldDomain(ToyWorldMultiAgentProblemInstance problem,
            double speed, Random random) {
        super(problem, speed, random);
    }

    @Override
    public JointSpatioTemporalState sampleState() {
        JointSpatioTemporalState sample = super.sampleState();

        // bias towards goal
        double t = sample.getTime();
        TimePoint[] states = sample.getAgentStates();
        for (int i = 0; i < sample.nAgents(); i++) {
            if (random.nextDouble() < strength) {
                states[i] = new TimePoint(problem.getTarget(i),t);
            }
        }

        return new JointSpatioTemporalState(states);
    }



}
