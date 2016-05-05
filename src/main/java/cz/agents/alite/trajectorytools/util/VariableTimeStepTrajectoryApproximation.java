package cz.agents.alite.trajectorytools.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public class VariableTimeStepTrajectoryApproximation {
	
	/**
	 * Approximate trajectory with samples of variable time step.
	 * @param trajectory Trajectory to be approximated.
	 * @param nfZonez 
	 * @param minTime Start time of the approximation.
	 * @param maxTime End time of the approximation.
	 * @param sampleStep Minimal sample step.
	 * @param maxAngle Maximal angle which is ignored. 
	 * @return
	 */

	public static List<TimePoint> approximate(Trajectory trajectory, Collection<SpaceTimeRegion> regions){
		return approximate(trajectory, trajectory.getMinTime(), trajectory.getMaxTime(), 0.5, Math.PI/8, regions);
	}

	public static List<TimePoint> approximate(Trajectory trajectory, double minTime, double maxTime, double sampleStep, double maxAngle){
		return approximate(trajectory, minTime, maxTime, sampleStep, maxAngle, new ArrayList<SpaceTimeRegion>());
	}

	public static List<TimePoint> approximate(Trajectory trajectory, double minTime, double maxTime, double sampleStep, double maxAngle, Collection<SpaceTimeRegion> regions){
		
		List<TimePoint> output = new LinkedList<TimePoint>();
		
		OrientedPoint prev = trajectory.getPosition(minTime);
		
		double prevt = minTime;
		
		//add first
		output.add(new TimePoint(prev,prevt));
		
		//add sampled
		for(double t = minTime+sampleStep; t < maxTime; t += sampleStep){
			OrientedPoint cur = trajectory.getPosition(t);
			
			if(Math.abs(prev.orientation.angle(cur.orientation)) > maxAngle
					|| isInCollision(new TimePoint(prev, prevt), new TimePoint(cur, t), regions)){
				output.add(new TimePoint(cur,t));
				
				prev = cur;
				prevt = t;
			}
			
			
		}
		
		//add last
		OrientedPoint last = trajectory.getPosition(maxTime);
		output.add(new TimePoint(last,maxTime));
		
		return output;
		
	}

	private static boolean isInCollision(TimePoint p1, TimePoint p2, Collection<SpaceTimeRegion> regions) {
		for (SpaceTimeRegion region : regions) {
			if (region.intersectsLine(p1, p2)) {
				return true;
			}
		}
		return false;
	}

}
