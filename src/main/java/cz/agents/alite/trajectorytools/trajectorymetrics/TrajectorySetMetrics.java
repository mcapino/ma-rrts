package cz.agents.alite.trajectorytools.trajectorymetrics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class TrajectorySetMetrics {

    private static final int THRESHOLD = 100;
    private static final double SAMPLE_COUNT = 5000;
    
    private TrajectorySetMetrics() {}

    /**
     * See: 'Alexandra Coman: Generating Diverse Plans Using Quantitative and Qualitative Plan Distance Metrics'
     *
     * @param trajectories
     * @param distanceMetric
     * @return
     */
    public static <V extends SpatialPoint, E> double getPlanSetAvgDiversity(Collection<PlannedPath<V, E>> trajectories, TrajectoryMetric<V, E> distanceMetric) {
        if (trajectories.size() == 0) {
            return 0;
        }
        
        List<PlannedPath<V, E>> trajList = new ArrayList<PlannedPath<V,E>>(trajectories);
        Random random = new Random();
        if (trajectories.size() > THRESHOLD) {
            // count approx. value
            double value = 0;
            for ( int i = 0; i < SAMPLE_COUNT; i++ ) {
                PlannedPath<V, E> traj1 = trajList.get(random.nextInt(trajList.size()));
                PlannedPath<V, E> traj2 = trajList.get(random.nextInt(trajList.size()));
                value += distanceMetric.getTrajectoryDistance(traj1, traj2);
            }
            return value / SAMPLE_COUNT;
        } else {
            // count exact value
            double value = 0;
            for (PlannedPath<V, E> traj1 : trajectories) {
                for (PlannedPath<V, E> traj2 : trajectories) {
                    value += distanceMetric.getTrajectoryDistance(traj1, traj2);
                }
            }
            return value / ( trajectories.size() * trajectories.size() - 1 );
        }
    }

    public static <V extends SpatialPoint, E> double getRelativePlanSetAvgDiversity(PlannedPath<V, E> trajectory, Collection<PlannedPath<V, E>> trajectories, TrajectoryMetric<V, E> distanceMetric) {
        double value = 0;
        for (PlannedPath<V, E> otherTraj : trajectories) {
            value += distanceMetric.getTrajectoryDistance(trajectory, otherTraj);
        }
        return value / trajectories.size();
    }

    public static <V extends SpatialPoint, E> double getRelativePlanSetMaxDiversity(PlannedPath<V, E> trajectory, Collection<PlannedPath<V, E>> trajectories, TrajectoryMetric<V, E> distanceMetric) {
        double maxValue = 0;
        for (PlannedPath<V, E> otherTraj : trajectories) {
            maxValue = Math.max( maxValue, distanceMetric.getTrajectoryDistance(trajectory, otherTraj));
        }
        return maxValue;
    }

    public static <V extends SpatialPoint, E> double getRelativePlanSetMinDiversity(PlannedPath<V, E> trajectory, Collection<PlannedPath<V, E>> trajectories, TrajectoryMetric<V, E> distanceMetric) {
        double minValue = Double.MAX_VALUE;
        for (PlannedPath<V, E> otherTraj : trajectories) {
            minValue = Math.min( minValue, distanceMetric.getTrajectoryDistance(trajectory, otherTraj));
        }
        return minValue;
    }
    
    
    public static <V extends SpatialPoint, E> Collection<PlannedPath<V, E>> getBestPaths(
            Collection<PlannedPath<V, E>> plannedPaths, 
            int number, 
            TrajectoryMetric<V, E> metric) {
        if (plannedPaths.size() <= number || number < 0) {
            return plannedPaths;
        }
        
        List<PlannedPath<V, E>> paths = new ArrayList<PlannedPath<V, E>>();

        List<PlannedPath<V, E>> curPaths = new ArrayList<PlannedPath<V, E>>(plannedPaths);
        while (paths.size() < number) {

            double maxValue = -1;
            PlannedPath<V, E> maxPath = null;
            for (PlannedPath<V, E> path : curPaths) {
                double value = TrajectorySetMetrics.getRelativePlanSetAvgDiversity(path, curPaths, metric);
                if (maxValue < value) {
                    maxValue = value;
                    maxPath = path;
                }
            }

            paths.add(maxPath);
            curPaths.remove(maxPath);
        }
        
        return paths;
    }
}