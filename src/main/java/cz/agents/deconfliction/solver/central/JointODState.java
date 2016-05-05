package cz.agents.deconfliction.solver.central;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graphs;
import org.jgrapht.SingleEdgeGraphPath;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointODState {
    protected static final double SAMPLES_PER_SEPARATION = 20.0;
    protected ODState[] agents;
    protected JointODState parent;
    protected double g;
    protected double h;
    protected double f;
    protected int v = 0;
    
    public JointODState(Waypoint[] waypoints, JointODState parent, double g, double h) {
        this(ODState.fromWaypoints(waypoints), parent, g, h, 0);
    }
    
    public JointODState(Waypoint[] waypoints, JointODState parent, JointGoalState goal, int v) {
        this(waypoints, parent, 0.0, 0.0);
        this.g = getCost();
        this.h = getHeuristicEstimateTo(goal);
        this.f = this.g + this.h;
        this.v = v;
    }
    

    public JointODState(ODState[] agents, JointODState parent, double g, double h) {
        this(agents, parent, g, h, 0);
    }

    public JointODState(ODState[] agents, JointODState parent, double g, double h, int v) {
        this.agents = Arrays.copyOf(agents, agents.length);
        this.parent = parent;
        this.g = g;
        this.h = h;
        this.f = g + h;
        this.v = v;
    }

    public JointODState(ODState[] agents, JointODState parent, JointGoalState goal, int v) {
        this(agents, parent, 0.0, 0.0);
        this.g = getCost();
        this.h = getHeuristicEstimateTo(goal);
        this.f = this.g + this.h;
        this.v = v;
    }

    ODState getAgent(int n) {
        return agents[n];
    }

    public int nAgents() {
        return agents.length;
    }

    public JointODState getParent() {
        return parent;
    }

    public double getCost() {
        double total = 0.0;
        for (int i = 0; i < agents.length; i++) {
            if (agents[i] != null) {
                total += agents[i].getCost();
            }
        }
        return total;
    }

    public double getHeuristicEstimateTo(JointGoalState goal) {
        return getManhattanDistance(goal);
    }
    
    // Should be used as the heuristic function if the diagonal move costs the same as the sidways moves
	private double getLInfDistance(JointGoalState goal) {
		double total = 0.0;
        for (int i = 0; i < agents.length; i++) {
            // Straight line distance - can be changed to Manhattan
            if (agents[i] != null) {
                if (agents[i].getNextManeuver() == null) {
                    total += agents[i].getCurrentWaypoint().distanceLinf(goal.getAgent(i).getCurrentWaypoint());
                } else {
                    total += agents[i].nextWaypoint.distanceLinf(goal.getAgent(i).getCurrentWaypoint());
                }
            }
        }
        return total;
	}
	
	// Should be used as the heuristic function if the diagonal move costs sqrt(2) times more than the sideways move
	private double getEuclideanDistance(JointGoalState goal) {
		double total = 0.0;
        for (int i = 0; i < agents.length; i++) {
            // Straight line distance - can be changed to Manhattan
            if (agents[i] != null) {
                if (agents[i].getNextManeuver() == null) {
                    total += agents[i].getCurrentWaypoint().distance(goal.getAgent(i).getCurrentWaypoint());
                } else {
                    total += agents[i].nextWaypoint.distance(goal.getAgent(i).getCurrentWaypoint());
                }
            }
        }
        return total;
	}
	
	// Should be used as the heuristic function if diagonal moves are not allowed
	private double getManhattanDistance(JointGoalState goal) {
		double total = 0.0;
        for (int i = 0; i < agents.length; i++) {
            // Straight line distance - can be changed to Manhattan
            if (agents[i] != null) {
                if (agents[i].getNextManeuver() == null) {
                    total += agents[i].getCurrentWaypoint().distance(goal.getAgent(i).getCurrentWaypoint());
                } else {
                    total += agents[i].nextWaypoint.distance(goal.getAgent(i).getCurrentWaypoint());
                }
            }
        }
        return total;
	}

    public boolean isIntermediate() {
        for (int i = 0; i < agents.length; i++) {
            if (agents[i] != null && agents[i].getNextManeuver() != null) return true;
        }
        return false;
    }

    public boolean isFullyAssigned() {
        for (int i = 0; i < agents.length; i++) {
            if (agents[i] != null && agents[i].getNextManeuver() == null) return false;
        }
        return true;
    }

    /**
     *
     * @return id of last assigned, -1 if none is assigned yet
     */
    public int getLastAssignedId() {
        for (int i = agents.length-1; i >= 0; i--) {
            if (agents[i] != null && agents[i].getNextManeuver() != null) {
                return i;
            }
        }

        return (-1);
    }

    private int getNextAssignableId() {
        for (int i = getLastAssignedId()+1; i < agents.length; i++) {
            if (agents[i] != null && agents[i].getNextManeuver() == null) {
                return i;
            }
        }

        return (-1);
    }


    public List<JointODState> getChildren(
            DirectedGraph<Waypoint, SpatialManeuver>[] maneuvers, JointGoalState goal,
            double separation,
            Collection<Trajectory> hardConstrainingTrajectories,
            Collection<Trajectory> softConstrainingTrajectories) {
        List<JointODState> children = new LinkedList<JointODState>();

        // agents[i] is the first agent that has no maneuver assigned yet

        int i = -1;
        if (isFullyAssigned()) {
            // We are in a fully assigned state, skip
            return children;
        } else {
            i = getNextAssignableId();
        }

        Waypoint waypoint = agents[i].getCurrentWaypoint();
        Set<SpatialManeuver> nextManeuvers = new HashSet<SpatialManeuver>(maneuvers[i].outgoingEdgesOf(waypoint));

        for (SpatialManeuver nextManeuver : nextManeuvers) {
            Waypoint nextWaypoint = Graphs.getOppositeVertex(maneuvers[i], nextManeuver, agents[i].getCurrentWaypoint());
            if (isApplicable(maneuvers[i], i, nextManeuver, nextWaypoint, separation, hardConstrainingTrajectories)) {

                ODState[] newAgents = Arrays.copyOf(agents, agents.length);

                // The cost is the weight of the maneuver, except when the agent is on the goal location

                double maneuverCost = maneuvers[i].getEdgeWeight(nextManeuver);
                newAgents[i] = new ODState(agents[i].getCurrentWaypoint(), agents[i].getTime(), nextManeuver, nextWaypoint, agents[i].getCost() + maneuverCost);

                int softViolations = countSoftConstraintViolations(maneuvers[i], i, nextManeuver, nextWaypoint, separation, softConstrainingTrajectories);

                JointODState intermediateState = new JointODState(newAgents, this, goal, this.v + softViolations);
                children.add(intermediateState);

                // Check whether the new state is partially or fully assigned
                // If fully assigned generate also a standard state
                assert(i <= agents.length);

                if (intermediateState.isFullyAssigned()) {
                    // The new state is a fully assigned state
                    // Generate also a standard state
                    children.add(createStandardState(intermediateState, goal));
                }
            }
        }


        return children;
    }


    @Override
    public boolean equals(Object obj) {
        // !!! Ignores time
        if (obj instanceof JointODState) {
            JointODState other = (JointODState) obj;
            if (this.nAgents() == other.nAgents()) {
                for (int i=0; i < agents.length; i++) {
                    if (agents[i] != null) {
                        if (!agents[i].equalsIgnoreTime(other.getAgent(i))) {
                            return false;
                        }
                    } else {
                        // agents[i] == null
                        if (other.getAgent(i) != null) {
                            return false;
                        }
                    }
                }
                return true;
            }
        }
        return false;
    }

    private static JointODState createStandardState(JointODState fullyAssignedState, JointGoalState goal) {

        ODState[] newAgents = new ODState[fullyAssignedState.nAgents()];

        for (int i=0; i < fullyAssignedState.nAgents(); i++) {
            if (fullyAssignedState.getAgent(i) != null) {
                Waypoint newWaypoint = fullyAssignedState.getAgent(i).getNextWaypoint();
                double newTime = fullyAssignedState.getAgent(i).getTime() + fullyAssignedState.getAgent(i).getNextManeuver().getDuration();
                double cost = fullyAssignedState.getAgent(i).getCost();
                newAgents[i] = new ODState(newWaypoint, newTime, null, null, cost);
            }
        }

        return new JointODState(newAgents, fullyAssignedState, goal, fullyAssignedState.v);
    }

    public boolean isApplicable(DirectedGraph<Waypoint, SpatialManeuver> maneuvers, int candidateId, SpatialManeuver candidateManeuver, Waypoint candidateNextWaypoint, double separation, Collection<Trajectory> hardConstrainingTrajectories) {

        double tmin = this.getAgent(candidateId).getTime();
        double tmax = tmin + candidateManeuver.getDuration();
        List<Trajectory> constrainingTrajectories = getConstrainingTrajectories(maneuvers, this, candidateId, tmin, tmax);
        constrainingTrajectories.addAll(hardConstrainingTrajectories);

        Trajectory candidateTrajectory =
                new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(
                        agents[candidateId].getTime(),
                        new SingleEdgeGraphPath<Waypoint, SpatialManeuver>(maneuvers,
                                agents[candidateId].getCurrentWaypoint(),
                                candidateManeuver,
                                candidateNextWaypoint,
                                candidateManeuver.getDuration()),
                                candidateManeuver.getDuration()
                );

        return !SeparationDetector.hasConflict(candidateTrajectory, constrainingTrajectories, separation, separation / SAMPLES_PER_SEPARATION,1.0);
    }

    public int countSoftConstraintViolations(
            DirectedGraph<Waypoint, SpatialManeuver> maneuvers,
            int candidateId, SpatialManeuver candidateManeuver,
            Waypoint candidateNextWaypoint, double separation,
            Collection<Trajectory> softConstrainingTrajectories) {

        double tmin = this.getAgent(candidateId).getTime();
        double tmax = tmin + candidateManeuver.getDuration();
        List<Trajectory> constrainingTrajectories = getConstrainingTrajectories(maneuvers, this, candidateId, tmin, tmax);
        constrainingTrajectories.addAll(softConstrainingTrajectories);

        Trajectory candidateTrajectory =
                new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(
                        agents[candidateId].getTime(),
                        new SingleEdgeGraphPath<Waypoint, SpatialManeuver>(maneuvers,
                                agents[candidateId].getCurrentWaypoint(),
                                candidateManeuver,
                                candidateNextWaypoint,
                                candidateManeuver.getDuration()),
                        candidateManeuver.getDuration()
                );

        return SeparationDetector.countConflictingTrajectories(candidateTrajectory, constrainingTrajectories, separation, separation / SAMPLES_PER_SEPARATION);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[");

        for (int i=0; i < agents.length; i++) {
            sb.append(agents[i]);
            sb.append(" ");
        }

        sb.append(" " + (new DecimalFormat("#.##")).format(f) + "]");

        return sb.toString();
    }

    /**
     * If the maneuvers used during the expansion have different duration
     * and a new maneuver is to going to be applied, the new maneuver (starting at tmin, lasting to tmax)
     * have to be checked against the parts of the trajectories of the higher priority agents
     * that timewise intersect. I.e. all maneuvers (starting at tmin', lasting to tmax') for which holds
     * tmin' <= tmax and tmax' >= tmin.
     */
    public static List<Trajectory> getConstrainingTrajectories(
            DirectedGraph<Waypoint, SpatialManeuver> maneuvers,
            JointODState state, int constrainedAgentId, double tmin,
            double tmax) {
        int nAgents = state.nAgents();
        LinkedList<SpatialManeuver>[] paths = new LinkedList[nAgents];
        double startTimes[] = new double[nAgents];
        Waypoint startWaypoints[] = new Waypoint[nAgents];
        Waypoint endWaypoints[] = new Waypoint[nAgents];

        for (int i=0; i < nAgents; i++) {
            if (state.getAgent(i) != null) {
                paths[i] = new LinkedList<SpatialManeuver>();
            }
        }

        for (int i=0; i < nAgents; i++) {
            startTimes[i] = 0.0;
        }

        JointODState current = state;
        while (current.getParent() != null) {
            if (current.isIntermediate()) {
                int i = current.getLastAssignedId();

                if (i != constrainedAgentId) {
                    SpatialManeuver maneuver = current.getAgent(i).getNextManeuver();
                    double startTime = current.getAgent(i).getTime();
                    Waypoint startWaypoint = current.getAgent(i).getCurrentWaypoint();
                    Waypoint endWaypoint = current.getAgent(i).getNextWaypoint();

                    if (startTime <= tmax && startTime + maneuver.getDuration() >= tmin) {
                        paths[i].addFirst(current.getAgent(i).getNextManeuver());
                        startTimes[i] = startTime;
                        startWaypoints[i] = startWaypoint;
                        if (endWaypoints[i] == null) {
                            endWaypoints[i] = endWaypoint;
                        }
                    }
                }
            }
            current = current.getParent();
        }

        List<Trajectory> trajectories = new LinkedList<Trajectory>();
        for (int i=0; i < paths.length; i++) {
            if (paths[i] != null && paths[i].size() > 0) {
                double duration = 0.0;
                for(SpatialManeuver maneuver : paths[i]) {
                    duration += maneuver.getDuration();
                }
                GraphPathImpl<Waypoint, SpatialManeuver> graphPath = new GraphPathImpl<Waypoint, SpatialManeuver>(maneuvers, startWaypoints[i], endWaypoints[i], paths[i], duration);
                trajectories
                        .add(new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(
                                startTimes[i], graphPath,
                                duration));
            }
        }

        return trajectories;
    }

    public double getEvaluation() {
        return f;
    }
    
    public Waypoint[] getWaypoints() {
    	Waypoint waypoints[] = new Waypoint[nAgents()];
    	for (int i=0; i<nAgents(); i++) {
    		waypoints[i] = agents[i].currentWaypoint;
    	}
    	return waypoints;
    }
}