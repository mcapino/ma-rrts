package cz.agents.alite.trajectorytools.graph.jointspatiotemporal;

import java.util.Arrays;

import cz.agents.alite.trajectorytools.util.TimePoint;

public class JointSpatioTemporalState {

    final TimePoint[] agentStates;

    public JointSpatioTemporalState(TimePoint[] agentStates) {
        this.agentStates = agentStates;
        assert(allTimesAreIdentical());
    }

    private boolean allTimesAreIdentical() {
        if (agentStates.length <= 1)
            return true;

        double time = agentStates[0].getTime();

        for (int i=0; i<0; i++) {
            if (agentStates[i].getTime() != time) {
                return false;
            }
        }

        return true;
    }

    public int nAgents() {
        return agentStates.length;
    }

    public TimePoint get(int n) {
        return agentStates[n];
    }

    public double sumOfSpatialDistances(JointSpatioTemporalState other) {
        double dist = 0.0;

        if (this.nAgents() != other.nAgents()) {
            throw new IllegalArgumentException("Joint states have different sizes.");
        }

        for (int i = 0 ; i < this.nAgents(); i++) {
            dist += this.get(i).getSpatialPoint().distance(other.get(i).getSpatialPoint());
        }

        return dist;
    }

    public double maxSpatialDistance(JointSpatioTemporalState other) {
        double dist = 0.0;

        if (this.nAgents() != other.nAgents()) {
            throw new IllegalArgumentException("Joint states have different sizes.");
        }

        for (int i = 0 ; i < this.nAgents(); i++) {
            double singleAgentDistance = this.get(i).getSpatialPoint().distance(other.get(i).getSpatialPoint());
            if (dist < singleAgentDistance) {
                dist = singleAgentDistance;
            }
        }

        return dist;
    }

    public double getTime() {
        return agentStates[0].getTime();
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Arrays.hashCode(agentStates);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        JointSpatioTemporalState other = (JointSpatioTemporalState) obj;
        if (!Arrays.equals(agentStates, other.agentStates))
            return false;
        return true;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();

        builder.append(String.format("%.2f ", agentStates[0].getTime()));

        for (int i = 0 ; i < this.nAgents(); i++) {
            builder.append(String.format("(%.2f, %.2f, %.2f) ", agentStates[i].x, agentStates[i].y, agentStates[i].z));
        }

        return builder.toString();
    }

    public TimePoint[] getAgentStates() {
        return agentStates;
    }

}
