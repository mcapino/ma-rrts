package cz.agents.deconfliction.util;

import java.util.LinkedList;
import java.util.List;

import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public class Trajectories {

    /**
     * Adds (and overwrites) all non-null elements from array b to array a.
     */
    public static EvaluatedTrajectory[] add(EvaluatedTrajectory[] a, EvaluatedTrajectory[] b){
        assert(a.length == b.length);
        for (int i = 0; i < a.length; i++) {
            if (b[i] != null) {
                a[i] = b[i];
            }
        }
        return a;
    }

    public static List<Trajectory> toList(EvaluatedTrajectory[] trajectories) {
        List<Trajectory> list = new LinkedList<Trajectory>();
        for (int i=0; i < trajectories.length; i++) {
            if (trajectories[i] != null) {
                list.add(trajectories[i]);
            }
        }
        return list;
    }

	public static double evaluateCost(EvaluatedTrajectory[] trajectories) {
        double cost = 0.0;
        for (EvaluatedTrajectory trajectory : trajectories) {
            if (trajectory != null) {
                cost += trajectory.getCost();
            }
        }
        return cost;
	} 

	public static Trajectory[] toTrajectoryArray(
			EvaluatedTrajectory[] trajectories) {
		Trajectory[] result = new Trajectory[trajectories.length];
		System.arraycopy(trajectories, 0, result, 0, trajectories.length);
		return result;
	}

}
