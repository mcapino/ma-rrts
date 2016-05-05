package cz.agents.alite.trajectorytools.graph.jointspatial;

import java.text.DecimalFormat;
import java.util.Arrays;

import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointWaypointTimeState {

    final Waypoint[] positions;
    double time;

    public JointWaypointTimeState(Waypoint[] positions, double time) {
        this.positions = positions;
        this.time = time;
    }

    public int nAgents() {
        return positions.length;
    }

    public Waypoint getPosition(int n) {
        return positions[n];
    }

    public double getTime() {
        return time;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Arrays.hashCode(positions);
        long temp;
        temp = Double.doubleToLongBits(time);
        result = prime * result + (int) (temp ^ (temp >>> 32));
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
        JointWaypointTimeState other = (JointWaypointTimeState) obj;
        if (!Arrays.equals(positions, other.positions))
            return false;
        if (!MathUtil.equals(getTime(), other.getTime(), 0.001))
            return false;
        return true;
    }

    public Waypoint[] getPositions() {
        return positions;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[@" + (new DecimalFormat("#.##")).format(time) +" ");

        for (int i=0; i < positions.length; i++) {
            sb.append(positions[i]);
            sb.append(", ");
        }

        sb.delete(sb.length()-2, sb.length());

        sb.append("]");

        return sb.toString();
    }
    /**
     * Return lower bound on duration of transition between this state and other state.
     * Assumes grid.
     */
    public double minDuration(JointWaypointTimeState other, double speed) {
        return minDuration(other.getPositions(), speed);
    }

    public double minDuration(Waypoint[] other, double speed) {
        double longestDistance = 0.0;
        assert(this.nAgents() == other.length);
        for (int i=0; i < this.nAgents(); i++) {
            assert(getPosition(i) != null && other[i]!= null);
            if (positions[i] != null) {
                double distance = getPosition(i).distanceL1(other[i]);
                if (distance > longestDistance) {
                    longestDistance = distance;
                }
            }
        }
        return longestDistance/speed;
    }

    /**
     *  Returns lower bound on the cost of paths between this state and other state.
     */
    public double minCostDistance(JointWaypointTimeState other, double speed) {
        return minCostDistance(other.getPositions(), speed);
    }


    /**
     *  Returns lower bound on the cost of paths between this state and other state.
     */
    public double minCostDistance(Waypoint[] other, double speed) {
        double minCost = 0;
        assert(this.nAgents() == other.length);
        for (int i=0; i < this.nAgents(); i++) {
            assert(getPosition(i) != null && other[i] != null);
            if (positions[i] != null) {
                minCost += singleAgentMinCostDistance(positions[i], other[i], speed);
            }
        }
        return minCost;
    }
    
    public double lowerBoundOnSumDistanceTo(Waypoint[] other) {
        double minDist = 0;
        assert(this.nAgents() == other.length);
        for (int i=0; i < this.nAgents(); i++) {
            assert(getPosition(i) != null && other[i] != null);
            if (positions[i] != null) {
                minDist += positions[i].distanceL1(other[i]);
            }
        }
        return minDist;
    }
    

    /**
     * Returns lower bound on cost of path between two waypoints.
     * Assumes the cost to be the time spent outside the goal position. Further, asssumes a grid,
     * therefore uses manhattan distance.
     */
    static public double singleAgentMinCostDistance(Waypoint start, Waypoint end, double speed) {
        // !!!! Manhattan
        return start.distanceL1(end) / speed;
    }
}
