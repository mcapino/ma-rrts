package cz.agents.deconfliction.solver.central.jaastar;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graphs;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointState {
    protected static final double SAMPLES_PER_SEPARATION = 20.0;
    protected Waypoint[] agents;
    protected double time;
    protected JointState parent;
    protected JointAction edgeFromParent;
    protected double g;
    protected double h;
    protected double f;
    protected int v = 0;
    
    static class TimeoutExceededException extends RuntimeException  {};

    public JointState(Waypoint[] agents, double time, JointState parent, JointAction edgeFromParent, double g, double h) {
        this.agents = Arrays.copyOf(agents, agents.length);
        this.parent = parent;
        this.edgeFromParent = edgeFromParent;
        this.g = g;
        this.h = h;
        this.f = g + h;
        this.time = time;
    }

    public JointState(Waypoint[] agents, double time, JointState parent, JointAction edgeFromParent, double cost, Waypoint[] goal) {
        this(agents, time, parent, edgeFromParent, 0.0, 0.0);
        this.g = cost;
        this.h = getHeuristicEstimateTo(goal);
        this.f = this.g + this.h;
        this.v = 0;
    }

    Waypoint getAgent(int n) {
        return agents[n];
    }

    public int nAgents() {
        return agents.length;
    }

    public JointState getParent() {
        return parent;
    }

    public double getCost() {
        return h;
    }

    public double getHeuristicEstimateTo(Waypoint[] goal) {
        return getDistance(goal);
    }

    private double getDistance(Waypoint[] goal) {
        double total = 0.0;
        for (int i = 0; i < agents.length; i++) {
            // Straight line distance - can be changed to Manhattan
            if (agents[i] != null) {
                    total += getSingleAgentDistance(agents[i], goal[i]);
            }
        }
        return total;
    }

    double getSingleAgentDistance(Waypoint w1, Waypoint w2) {
        return w1.distanceL1(w2);
    }

    public List<JointState> getChildren(
            DirectedGraph<Waypoint, SpatialManeuver>[] maneuvers, Waypoint[] goal,
            double separation,
            Collection<Trajectory> hardConstrainingTrajectories,
            Collection<Trajectory> softConstrainingTrajectories,
            long interruptAtNs) {
    	List<JointState> children = new LinkedList<JointState>();

        Collection<JointAction> jointActions = generateJointActions(maneuvers, separation, hardConstrainingTrajectories, interruptAtNs);

        for (JointAction jointAction : jointActions) {
            Waypoint[] newWaypoints = jointAction.getTargets();

            children.add(new JointState(newWaypoints, time+1.0, this, jointAction, this.g + jointAction.getCost(), goal));
        }

        return children;
    }

	Collection<JointAction> generateJointActions(
			DirectedGraph<Waypoint, SpatialManeuver>[] maneuvers,
			double separation, Collection<Trajectory> hardConstraints,
			long interruptAtNs) {
        List<JointAction> jointActions = new LinkedList<JointAction>();

        if (interruptAtNs < System.nanoTime()) {
        	throw new TimeoutExceededException();
        }
        
        if (nAgents() == 0) {
            return jointActions;
        }

        // determine applicable maneuvers
        Set<SpatialManeuver> nextManeuvers = new HashSet<SpatialManeuver>(maneuvers[0].outgoingEdgesOf(getAgent(0)));

        for (SpatialManeuver candidateManeuver : nextManeuvers) {
            if (isManeuverApplicable(candidateManeuver, hardConstraints, separation)) {
                // maneuver is applicable
                Waypoint target = Graphs.getOppositeVertex(maneuvers[0], candidateManeuver, getAgent(0));

                SpatialManeuver[] actions = new SpatialManeuver[nAgents()];
                Waypoint[] targets = new Waypoint[nAgents()];
                double[] costs = new double[nAgents()];

                actions[0] = candidateManeuver;
                targets[0] = target;
                costs[0] = maneuvers[0].getEdgeWeight(candidateManeuver);

                jointActions.add(new JointAction(actions, targets, costs));
            }
        }

        if (nAgents() == 1) {
            return jointActions;
        } else {
            return generateJointActions(jointActions, maneuvers, 1, separation, hardConstraints, interruptAtNs);
        }

    }

	Collection<JointAction> generateJointActions(
			Collection<JointAction> inputJointActions,
			DirectedGraph<Waypoint, SpatialManeuver>[] maneuvers, int i,
			double separation, Collection<Trajectory> hardConstraints,
			long interruptAtNs) {
		
        List<JointAction> outputJointActions = new LinkedList<JointAction>();
        for (JointAction jointAction : inputJointActions) {
            
        	if (interruptAtNs < System.nanoTime()) {
            	throw new TimeoutExceededException();
            }

        	List<Trajectory> constraints = jointAction.getTrajectoryList(time);
            constraints.addAll(hardConstraints);

            // determine applicable maneuvers
            Set<SpatialManeuver> nextManeuvers = new HashSet<SpatialManeuver>(maneuvers[i].outgoingEdgesOf(this.getAgent(i)));

            for (SpatialManeuver candidateManeuver : nextManeuvers) {
                if (isManeuverApplicable(candidateManeuver, constraints, separation)) {
                    // maneuver is applicable
                    Waypoint target = Graphs.getOppositeVertex(maneuvers[i], candidateManeuver, getAgent(i));
                    assert(target != null);
                    outputJointActions.add(JointAction.addActionForAgent(jointAction, i, candidateManeuver, target, maneuvers[i].getEdgeWeight(candidateManeuver)));
                }
            }
        }

        if (i == nAgents()-1) {
            return outputJointActions;
        } else {
            return generateJointActions(outputJointActions, maneuvers, i+1, separation, hardConstraints, interruptAtNs);
        }

    }

    private boolean isManeuverApplicable(SpatialManeuver candidateManeuver, Collection<Trajectory> trajectories, double separation){
        return !SeparationDetector.hasConflict(candidateManeuver.getTrajectory(time), trajectories, separation, separation/SAMPLES_PER_SEPARATION,1.0);
    }

    public boolean areAgentsAt(Waypoint[] waypoints) {
        return Arrays.equals(agents,waypoints);
    }

    @Override
    public boolean equals(Object obj) {
        
        if (obj instanceof JointState) {
            JointState other = (JointState) obj;
            if (this.nAgents() == other.nAgents()) {
                for (int i=0; i < agents.length; i++) {
                    if (agents[i] != null) {
                        if (!agents[i].equals(other.getAgent(i))) {
                            return false;
                        }
                    } else {
                        // agents[i] == null
                        if (other.getAgent(i) != null) {
                            return false;
                        }
                    }
                }

                if (MathUtil.equals(this.time, other.time, 0.001)) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[" + String.format("@%.2f ", time));

        for (int i=0; i < agents.length; i++) {
            sb.append(agents[i]);
            sb.append(" ");
        }

        sb.append(String.format(" f: %.2f] ", f));

        return sb.toString();
    }

    public double getEvaluation() {
        return f;
    }

    public Waypoint[] getWaypoints() {
        return agents;
    }

    public JointAction getEdgeFromParent() {
        return edgeFromParent;
    }
}