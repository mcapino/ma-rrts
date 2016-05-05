package cz.agents.alite.trajectorytools.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public class SeparationDetector {

    /**
     * Computes pairwise conflicts between thisTrajectory and otherTrajectories.
     */
    public static List<TimePoint> computePairwiseConflicts(
            Trajectory thisTrajectory,
            Collection<Trajectory> otherTrajectoriesCollection,
            double separation, double samplingInterval) {


        List<Trajectory> otherTrajectories = new ArrayList<Trajectory>(otherTrajectoriesCollection);
        List<TimePoint> conflicts = new LinkedList<TimePoint>();

        for (double t = thisTrajectory.getMinTime(); t < thisTrajectory.getMaxTime(); t += samplingInterval) {
            OrientedPoint thisTrajectoryPos = thisTrajectory.getPosition(t);
            for (int j = 0; j < otherTrajectories.size(); j++) {

                if (otherTrajectories.get(j) != null) {
                    if (t >= otherTrajectories.get(j).getMinTime() && t <= otherTrajectories.get(j).getMaxTime()) {
                        OrientedPoint otherTrajectoryPos = otherTrajectories.get(j).getPosition(t);
                        if (thisTrajectoryPos.distance(otherTrajectoryPos) <= separation) {
                            conflicts.add(new TimePoint(thisTrajectoryPos, t));
                        }
                    }
                }
            }
        }
        return conflicts;
    }
    
    public static boolean hasAnyPairwiseConflict(
            Trajectory thisTrajectory,
            Collection<Trajectory> otherTrajectoriesCollection,
            double separation, double samplingInterval) {


        List<Trajectory> otherTrajectories = new ArrayList<Trajectory>(otherTrajectoriesCollection);

        for (double t = thisTrajectory.getMinTime(); t < thisTrajectory.getMaxTime(); t += samplingInterval) {
            OrientedPoint thisTrajectoryPos = thisTrajectory.getPosition(t);
            for (int j = 0; j < otherTrajectories.size(); j++) {

                if (otherTrajectories.get(j) != null) {
                    if (t >= otherTrajectories.get(j).getMinTime() && t <= otherTrajectories.get(j).getMaxTime()) {
                        OrientedPoint otherTrajectoryPos = otherTrajectories.get(j).getPosition(t);
                        if (thisTrajectoryPos.distance(otherTrajectoryPos) <= separation) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /**
     * Computes all pairwise conflicts between all pairs of trajectories from trajectoriesCollection.
     */
    public static List<TimePoint> computeAllPairwiseConflicts(Collection<Trajectory> trajectoriesCollection, double separation, double samplingInterval) {

        List<Trajectory> trajectories = new ArrayList<Trajectory>(trajectoriesCollection);
        List<TimePoint> conflicts = new LinkedList<TimePoint>();

        double minTime = Double.POSITIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMinTime() < minTime) {
                minTime = trajectory.getMinTime();
            }
        }

        double maxTime = Double.NEGATIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMaxTime() > maxTime) {
                maxTime = trajectory.getMaxTime();
            }
        }

        // iterate over all time points
        for (double t = minTime; t <= maxTime; t += samplingInterval) {
            // check all pairs of agents for conflicts at timepoint t
            for(int j=0; j < trajectories.size(); j++) {
                for (int k=j+1; k < trajectories.size(); k++) {
                    // check the distance between j and k
                    Trajectory a = trajectories.get(j);
                    Trajectory b = trajectories.get(k);

                    if (t >= a.getMinTime() && t <= a.getMaxTime() &&
                        t >= b.getMinTime() && t <= b.getMaxTime()) {
                        if (a.getPosition(t).distance(b.getPosition(t)) <= separation) {
                            conflicts.add(new TimePoint(a.getPosition(t), t));
                            conflicts.add(new TimePoint(b.getPosition(t), t));
                        }
                    }

                }
            }
        }
        return conflicts;
    }


    public static boolean hasAnyPairwiseConflict(Collection<Trajectory> trajectoriesCollection, double separation, double samplingInterval) {

        List<Trajectory> trajectories = new ArrayList<Trajectory>(trajectoriesCollection);

        double minTime = Double.POSITIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMinTime() < minTime) {
                minTime = trajectory.getMinTime();
            }
        }

        double maxTime = Double.NEGATIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMaxTime() > maxTime) {
                maxTime = trajectory.getMaxTime();
            }
        }

        // iterate over all time points
        for (double t = minTime; t <= maxTime; t += samplingInterval) {
            // check all pairs of agents for conflicts at timepoint t
            for(int j=0; j < trajectories.size(); j++) {
                for (int k=j+1; k < trajectories.size(); k++) {
                    // check the distance between j and k
                    Trajectory a = trajectories.get(j);
                    Trajectory b = trajectories.get(k);

                    if (t >= a.getMinTime() && t <= a.getMaxTime() &&
                        t >= b.getMinTime() && t <= b.getMaxTime()) {
                        if (a.getPosition(t).distance(b.getPosition(t)) <= separation) {
                            return true;
                        }
                    }

                }
            }
        }
        return false;
    }

    /**
     * finds the first conflicts and returns ids of the two agents involved in the conflict
     * @return array containing ids of agents involved in the conflict, null if no conflicts are found
     */
    public static int[] findFirstConflict(Collection<Trajectory> trajectoriesCollection, double separation, double samplingInterval) {

        List<Trajectory> trajectories = new ArrayList<Trajectory>(trajectoriesCollection);

        double minTime = Double.POSITIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMinTime() < minTime) {
                minTime = trajectory.getMinTime();
            }
        }

        double maxTime = Double.NEGATIVE_INFINITY;
        for (Trajectory trajectory : trajectories) {
            if (trajectory.getMaxTime() > maxTime) {
                maxTime = trajectory.getMaxTime();
            }
        }

        // iterate over all time points
        for (double t = minTime; t <= maxTime; t += samplingInterval) {
            // check all pairs of agents for conflicts at timepoint t
            for(int j=0; j < trajectories.size(); j++) {
                for (int k=j+1; k < trajectories.size(); k++) {
                    // check the distance between j and k
                    Trajectory a = trajectories.get(j);
                    Trajectory b = trajectories.get(k);

                    if (t >= a.getMinTime() && t <= a.getMaxTime() &&
                        t >= b.getMinTime() && t <= b.getMaxTime()) {
                        if (a.getPosition(t).distance(b.getPosition(t)) <= separation) {
                            return new int[] {j,k};
                        }
                    }

                }
            }
        }

        return null;
    }

    public static boolean hasConflict(Collection<Trajectory> trajectoriesCollection, double separation, double samplingInterval) {
        return hasAnyPairwiseConflict(trajectoriesCollection, separation, samplingInterval);
    }

    public static boolean hasConflict(Collection<Trajectory> trajectoriesCollection, double separation, double samplingInterval, double maxSpeed) {
        if(mayHaveConflict(trajectoriesCollection, separation, maxSpeed)){
            return hasAnyPairwiseConflict(trajectoriesCollection, separation, samplingInterval);
        }else{
            return false;
        }

    }

    /** Quickly determines if the trajectories are close enough for a conflict to occur. */
    public static boolean mayHaveConflict(Collection<Trajectory> trajectoriesCollection, double separation, double maxSpeed) {
        if(trajectoriesCollection.isEmpty())return false;

        List<Trajectory> trajectories = new ArrayList<Trajectory>(trajectoriesCollection);

        Trajectory t = trajectories.get(0);

        double criticalDist = (t.getMaxTime()-t.getMinTime())*maxSpeed + separation;

        for(int j=0; j < trajectories.size(); j++) {
            for (int k=j+1; k < trajectories.size(); k++) {
                // check the distance between j and k
                Trajectory a = trajectories.get(j);
                Trajectory b = trajectories.get(k);

                if(a.getPosition(a.getMinTime()).distance(b.getPosition(b.getMinTime())) < criticalDist){
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Determines if thisTrajectory has conflict with any of the trajectories from otherTrajectoriesCollection.
     */
    public static boolean hasConflict(Trajectory thisTrajectory, Collection<Trajectory> otherTrajectoriesCollection, double separation, double samplingInterval) {

        assert(thisTrajectory != null);
        assert(!otherTrajectoriesCollection.contains(null));

        List<Trajectory> otherTrajectories = new ArrayList<Trajectory>(otherTrajectoriesCollection);

        for (double t = thisTrajectory.getMinTime(); t < thisTrajectory.getMaxTime(); t += samplingInterval) {
            OrientedPoint thisTrajectoryPos = thisTrajectory.getPosition(t);
            for (int j = 0; j < otherTrajectories.size(); j++) {

                if (otherTrajectories.get(j) != null) {
                    if (t >= otherTrajectories.get(j).getMinTime() && t <= otherTrajectories.get(j).getMaxTime()) {
                        OrientedPoint otherTrajectoryPos = otherTrajectories.get(j).getPosition(t);
                        if (thisTrajectoryPos.distance(otherTrajectoryPos) <= separation) {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }
    
    /**
     * Determines if thisTrajectory has conflict with any of the trajectories from otherTrajectoriesCollection.
     */
    public static boolean hasConflict(Trajectory thisTrajectory, Collection<Trajectory> otherTrajectoriesCollection, double separation, double samplingInterval, double maxSpeed) {

        assert(thisTrajectory != null);
        assert(!otherTrajectoriesCollection.contains(null));

        List<Trajectory> otherTrajectories = new ArrayList<Trajectory>(otherTrajectoriesCollection);
        
        double criticalDist = (thisTrajectory.getMaxTime()-thisTrajectory.getMinTime())*maxSpeed + separation;
        
        boolean approximateConflict = false;
        
        for (int j = 0; j < otherTrajectories.size(); j++) {
        	Trajectory b = otherTrajectories.get(j); 
            if (b != null) {
            	if(thisTrajectory.getPosition(thisTrajectory.getMinTime()).distance(b.getPosition(b.getMinTime())) < criticalDist){
            		approximateConflict = true;
    			}
            }
        }
        
        if(approximateConflict){
	        for (double t = thisTrajectory.getMinTime(); t < thisTrajectory.getMaxTime(); t += samplingInterval) {
	            OrientedPoint thisTrajectoryPos = thisTrajectory.getPosition(t);
	            for (int j = 0; j < otherTrajectories.size(); j++) {
	
	                if (otherTrajectories.get(j) != null) {
	                    if (t >= otherTrajectories.get(j).getMinTime() && t <= otherTrajectories.get(j).getMaxTime()) {
	                        OrientedPoint otherTrajectoryPos = otherTrajectories.get(j).getPosition(t);
	                        if (thisTrajectoryPos.distance(otherTrajectoryPos) <= separation) {
	                            return true;
	                        }
	                    }
	                }
	            }
	        }
        }

        return false;
    }

    /**
     *  Counts the number of trajectories from the given collection that have a conflict with the given trajectory
     */
    public static int countConflictingTrajectories(Trajectory thisTrajectory, Collection<Trajectory> otherTrajectoriesCollection, double separation, double samplingInterval) {

        int count = 0;

        assert(thisTrajectory != null);
        assert(!otherTrajectoriesCollection.contains(null));

        List<Trajectory> otherTrajectories = new ArrayList<Trajectory>(otherTrajectoriesCollection);

        for (double t = thisTrajectory.getMinTime(); t < thisTrajectory.getMaxTime(); t += samplingInterval) {
            OrientedPoint thisTrajectoryPos = thisTrajectory.getPosition(t);
            for (int j = 0; j < otherTrajectories.size(); j++) {

                if (otherTrajectories.get(j) != null) {
                    if (t >= otherTrajectories.get(j).getMinTime() && t <= otherTrajectories.get(j).getMaxTime()) {
                        OrientedPoint otherTrajectoryPos = otherTrajectories.get(j).getPosition(t);
                        if (thisTrajectoryPos.distance(otherTrajectoryPos) <= separation) {
                            count++;
                            continue;
                        }
                    }
                }
            }
        }

        return count;
    }

}
