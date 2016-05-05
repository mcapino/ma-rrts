package cz.agents.deconfliction.solver.central;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.SolutionNotFoundException;

public class IDSolver extends CentralCooperativePathfindingSolver {

    Logger LOGGER = Logger.getLogger(IDSolver.class);

    final protected DirectedGraph<Waypoint, SpatialManeuver> maneuvers;
    final protected JointODState start;
    final protected JointGoalState goal;
    final protected int nAgents;
    final protected int maximumGroupSize;


    List<Group> groups = new LinkedList<Group>();

    public IDSolver(JointODState start, JointGoalState goal, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax) {
        this(start, goal, maneuvers, Integer.MAX_VALUE, separation, maxTime, vmax);
    }


    public IDSolver(ODState starts[], Waypoint ends[], DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax) {
        this(starts, ends, maneuvers, Integer.MAX_VALUE, separation, maxTime, vmax);
    }

    public IDSolver(JointODState start, JointGoalState goal, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, int maximumGroupSize, double separation, double maxTime, double vmax) {
        super(maneuvers, goal.getWaypoints(), separation, maxTime, vmax);
        this.start = start;
        this.goal = goal;
        this.nAgents = start.nAgents();
        this.maximumGroupSize = maximumGroupSize;
        this.maneuvers = maneuvers;
    }

    public IDSolver(ODState starts[], Waypoint ends[], DirectedGraph<Waypoint, SpatialManeuver> maneuvers, int maximumGroupSize, double separation, double maxTime, double vmax) {
        super(maneuvers, ends, separation, maxTime, vmax);

        if (starts.length != ends.length) {
            throw new IllegalArgumentException("Start waypoints and end waypoints have different length.");
        }

        this.goal = new JointGoalState(ends);
        this.start = new JointODState(starts, null, this.goal, 0);
        this.nAgents = starts.length;
        this.maximumGroupSize = maximumGroupSize;
        this.maneuvers = maneuvers;
    }

    public SearchResult solve(long runtimeLimitMs) {
        long interruptAt = System.currentTimeMillis() + runtimeLimitMs;
        boolean finished;
        finished = createSingletonGroups(interruptAt - System.currentTimeMillis(), true);
        if (finished) {
             finished = resolveConflicts(interruptAt - System.currentTimeMillis());
             if (finished) {
                 return new SearchResult(getTrajectoriesFromAllGroups(), true);
             }
        }

        // terminated prematurely
        return new SearchResult(null, false);
     }

    EvaluatedTrajectory[] getTrajectoriesFromAllGroups() {
         EvaluatedTrajectory[] trajectories = new EvaluatedTrajectory[nAgents];
         for (Group group : groups) {
             add(trajectories, group.getTrajectories());
         }
         return trajectories;
    }

    protected boolean createSingletonGroups(long runtimeLimitMs, boolean optimized) {
        boolean finished = false;
        long interruptAtMs = System.currentTimeMillis() + runtimeLimitMs;

        // create singleton groups
        for (int i = 0; i < nAgents; i++) {
            if (start.getAgent(i) != null) {

                ODState[] startAgentStates = new ODState[nAgents];
                startAgentStates[i] = new ODState(start.getAgent(i)
                        .getCurrentWaypoint(), start.getAgent(i).getTime(), null, null, 0);

                Waypoint[] goalWaypoints = new Waypoint[nAgents];
                goalWaypoints[i] = goal.getAgent(i).getCurrentWaypoint();

                // The lower bound is the heuristic estimate of the cost
                double h = (new JointODState(startAgentStates, null, 0, 0))
                            .getHeuristicEstimateTo(new JointGoalState(goalWaypoints));
                groups.add(new Group(startAgentStates, goalWaypoints, h));
            }
        }

        // Plan trajectories for the singleton groups
        int nIterations = optimized ? 2 : 1;
        for (int iteration = 0; iteration < nIterations; iteration++) {
	        for (Group group : groups) {
	        	
	        	LinkedList<Trajectory> softConstraints = new LinkedList<Trajectory>();
	        	if (optimized) {
	        		for (Group otherGroup : groups) {
	        			assert(otherGroup.getSize() == 1);
	        			if (!otherGroup.equals(group) && otherGroup.getTrajectories() != null) {
	        				for (Trajectory traj : otherGroup.getTrajectories()) {
	        					if (traj != null) {
	        						softConstraints.add(traj);
	        					}
	        				}
	        			}
	        		}
	        	}
	        	
	            SearchResult result = group.planOptimalTrajectories(maneuvers,
	                    separation, maxTime, vmax, Double.POSITIVE_INFINITY,
	                    new LinkedList<Trajectory>(), softConstraints, interruptAtMs - System.currentTimeMillis());
	
	            if (result.isFinished()) {
	                if (!result.foundSolution()) {
	                    throw new SolutionNotFoundException();
	                }
	            } else {
	                return false;
	            }
	        }
        }

        return true;

    }
  
    protected boolean resolveConflicts(long runtimeLimitMs) {

        long interruptAtMs = System.currentTimeMillis() + runtimeLimitMs;

        for (Group group : groups) {
            group.resetConflictedBefore();
        }

        int[] conflicts;
        while ((conflicts = detectFirstConflict(groups, separation)) != null) {

            // there are conflicts, find alternative (optimal) paths or join the two groups

            Group g1 = getGroupContainingAgent(conflicts[0]);
            Group g2 = getGroupContainingAgent(conflicts[1]);
            assert (g1 != g2);

            LOGGER.debug("Groups " + g1 + " and " + g2+ " are in conflict. Trying to resolve...");

            boolean foundAlternativePaths = false;
            if (!g1.conflictedBeforeWith(g2) && !g2.conflictedBeforeWith(g1)) {
                g1.setConflictedWith(g2);
                g2.setConflictedWith(g1);

                EvaluatedTrajectory[] originalG1Trajectories = Arrays.copyOf(g1.getTrajectories(),g1.getTrajectories().length);

                EvaluatedTrajectory[] g1Trajectories = null;

                if (g1.getSize() + g2.getSize() <= maximumGroupSize) {
                    // find optimal paths
                    SearchResult result = g1.planOptimalTrajectories(maneuvers,
                            separation, maxTime, vmax, g1.getCost(),
                            toList(g2.getTrajectories()),
                            generateSoftConstrainingTrajectories(groups, g1, g2), interruptAtMs - System.currentTimeMillis());

                    if (result.isFinished()) {
                        g1Trajectories = result.getTrajectories();
                    } else {
                        return false;
                    }

                    if (g1Trajectories != null) {
                        LOGGER.debug("Found different optimal-cost paths for group " + g1 + ". Avoided joining with " + g2 + "." );
                    } else {
                        LOGGER.debug("There are no alternative optimal-cost paths for group " + g1 + "." );
                    }

                } else {
                    // find any paths
                    SearchResult result = g1.planAnyTrajectories(maneuvers,
                            separation, maxTime, vmax,
                            toList(g2.getTrajectories()),
                            generateSoftConstrainingTrajectories(groups, g1, g2), interruptAtMs - System.currentTimeMillis());

                    if (result.isFinished()) {
                        g1Trajectories = result.getTrajectories();
                    } else {
                        return false;
                    }

                    if (g1Trajectories != null) {
                        LOGGER.debug("Found different (any-cost) paths for group " + g1 + ". Avoided joining with " + g2 + "." );
                    } else {
                        LOGGER.info("There are no consistent (any-cost) paths for group " + g1 + "." );
                    }

                }

                if (g1Trajectories != null) {
                    foundAlternativePaths = true;
                } else {
                    g1.revertTo(originalG1Trajectories);
                    EvaluatedTrajectory[] g2Trajectories = null;

                    if (g1.getSize() + g2.getSize() <= maximumGroupSize) {
                        // find optimal
                        SearchResult result = g2.planOptimalTrajectories(
                                maneuvers, separation, maxTime, vmax,
                                g2.getCost(), toList(g1.getTrajectories()),
                                generateSoftConstrainingTrajectories(groups, g1, g2), interruptAtMs - System.currentTimeMillis());

                        if (result.isFinished()) {
                            g2Trajectories = result.getTrajectories();
                        } else {
                            return false;
                        }

                        if (g2Trajectories != null) {
                            LOGGER.debug("Found different optimal-cost paths for group " + g2 + ". Avoided joining with " + g1 + "." );
                        } else {
                            LOGGER.debug("There are no alternative optimal-cost paths for group " + g2 + "." );
                        }
                    } else {
                        SearchResult result = g2.planAnyTrajectories(
                                maneuvers, separation, maxTime, vmax,
                                toList(g1.getTrajectories()),
                                generateSoftConstrainingTrajectories(groups, g1, g2), interruptAtMs - System.currentTimeMillis());

                        if (result.isFinished()) {
                            g2Trajectories = result.getTrajectories();
                        } else {
                            return false;
                        }

                        if (g2Trajectories != null) {
                            LOGGER.debug("Found different (any-cost) paths for group " + g2 + ". Avoided joining with " + g1 + "." );
                        } else {
                            LOGGER.info("There are no consistent (any-cost) paths for group " + g2 + "." );
                        }                    }

                    if (g2Trajectories != null) {
                        foundAlternativePaths = true;
                    }
                }
            } else {
                LOGGER.debug("Groups " + g1 + " and " + g2+ " conflicted before. Joining...");
            }

            if (!foundAlternativePaths) {
                // Join the groups
                Group newGroup = Group.join(g1, g2);
                groups.remove(g1);
                groups.remove(g2);
                groups.add(newGroup);

                LOGGER.info("Failed to find alternative paths for groups " + g1 + " " + g2 + ".");
                LOGGER.debug("Joining two groups size " + g1.getSize() + " and " + g2.getSize() + ". Creating a new group of size " + newGroup.getSize() + ": " + newGroup);
                LOGGER.debug("Starting planning for the new group...");

                SearchResult result = newGroup.planOptimalTrajectories(
                        maneuvers, separation, maxTime, vmax,
                        Double.POSITIVE_INFINITY, new LinkedList<Trajectory>(),
                        generateSoftConstrainingTrajectories(groups, newGroup), interruptAtMs - System.currentTimeMillis());

                EvaluatedTrajectory[] groupTrajectories;
                if (result.isFinished()) {
                    groupTrajectories = result.getTrajectories();
                } else {
                    return false;
                }

                LOGGER.debug("Planning finished... We've got optimal paths for group: " + newGroup + ".");

                if (groupTrajectories == null) {
                    throw new SolutionNotFoundException();
                }
            }
        }

        return true;
    }

    protected int[] detectFirstConflict(List<Group> groups, double separation) {
        EvaluatedTrajectory[] trajectories = new EvaluatedTrajectory[nAgents];
        for (Group group : groups) {
            add(trajectories, group.getTrajectories());
        }
        return SeparationDetector.findFirstConflict(toList(trajectories), separation, separation / SAMPLES_PER_SEPARATION);
    }

    protected Group getGroupContainingAgent(int i) {
        for (Group group : groups) {
            if(group.containsAgent(i)) {
                return group;
            }
        }
        return null;
    }

    protected List<Group> getGroups() {
        return groups;
    };

    protected double getCost() {
        double cost = 0.0;
        for (Group group : groups) {
            cost += group.getCost();
        }
        return cost;
    }

    protected double getLowerBound() {
        double lowerBound = 0.0;
        for (Group group : groups) {
            lowerBound += group.getLowerBound();
        }
        return lowerBound;
    }

}
