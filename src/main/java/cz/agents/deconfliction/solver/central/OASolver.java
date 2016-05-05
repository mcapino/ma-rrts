package cz.agents.deconfliction.solver.central;

import java.util.LinkedList;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.MathUtil;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class OASolver extends IDSolver{
    Logger LOGGER = Logger.getLogger(OASolver.class);

    final long INF = Long.MAX_VALUE;

    private int maximumGroupSize = 1;
    private EvaluatedTrajectory[] bestTrajectories = null;

    public OASolver(ODState starts[], Waypoint ends[], DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double vmax) {
        super(starts, ends, maneuvers, separation, maxTime, vmax);
    }

    public SearchResult solve() {
        return solve(Long.MAX_VALUE);
    }

    @Override
    public SearchResult solve(long runtimeLimitMs) {
        SearchResult result = step(runtimeLimitMs);
        return result;
    }

    public SearchResult step(long runtimeLimitMs) {
        long interruptAtMs = System.currentTimeMillis() + runtimeLimitMs;

        if (bestTrajectories == null) {
            // get first feasible solution using MGS1
            IDSolver mgs1 = new IDSolver(start, goal, maneuvers, 1, separation, maxTime, vmax);
            SearchResult result = mgs1.solve(interruptAtMs - System.currentTimeMillis());
            if (result.isFinished()) {
                groups = mgs1.getGroups();
                bestTrajectories = result.getTrajectories();
                notifyNewSolution(bestTrajectories, false);
            } else {
                return new SearchResult(null, false);
            }
        }

        LOGGER.debug("Found MGS1 solution... " + groups);

        maximumGroupSize = 2;
        while (!MathUtil.equals(getLowerBound(), getCost(), 0.0001)) {
            LOGGER.debug(String.format("Starting MGS" + maximumGroupSize + " iteration. We've got %.2fms left", (double)(interruptAtMs - System.currentTimeMillis())));

            boolean someGroupsChanged;
            do {
                someGroupsChanged = false;

                // Iterate and find the first group whose cost can be improved, break when done
                LinkedList<Group> groupsCopy = new LinkedList<Group>(groups);
                for (Group group : groupsCopy) {
                    if (group.getLowerBound() < group.getCost()
                            && group.getSize() < maximumGroupSize) {

                        if (System.currentTimeMillis() > interruptAtMs) {
                            return new SearchResult(bestTrajectories, false);
                        }

                        SearchResult result = group.planOptimalTrajectories(maneuvers, separation, maxTime,
                                vmax, group.getCost(),
                                new LinkedList<Trajectory>(),
                                generateSoftConstrainingTrajectories(groups, group), interruptAtMs - System.currentTimeMillis());

                        if (result.isFinished()) {

                            if (result.foundSolution()) {
                                LOGGER.debug("Found a new optimal path for group "+group+".");
                            } else {
                                LOGGER.debug("There is no alternative optimal path for group "+group+".");
                            }

                            boolean finished = resolveConflicts(interruptAtMs - System.currentTimeMillis());
                            if (finished) {
                                bestTrajectories = getTrajectoriesFromAllGroups();
                                notifyNewSolution(bestTrajectories, false);
                                someGroupsChanged = true;
                                break;
                            } else {
                                return new SearchResult(bestTrajectories, false);
                            }
                        } else {
                            return new SearchResult(bestTrajectories, false);
                        }
                    }
                }
            } while (someGroupsChanged);

            maximumGroupSize++;
        }

        return new SearchResult(bestTrajectories, true);

    }

}
