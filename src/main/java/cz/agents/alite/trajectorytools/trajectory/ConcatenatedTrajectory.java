package cz.agents.alite.trajectorytools.trajectory;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;

import cz.agents.alite.trajectorytools.util.OrientedPoint;

/**
 * A wrapper that interprets a list of trajectories as one concatenated trajectory. The trajectories must align
 * in time in such way that two consecutive trajectories must end and start in the same time respectively.
 * @author stolba
 *
 */

public class ConcatenatedTrajectory implements Trajectory {
	
	private static final Logger LOGGER = Logger.getLogger(ConcatenatedTrajectory.class);
	
	private final LinkedList<Trajectory> trajectories;
	
	private final double minTime;
	private final double maxTime;
	
	public ConcatenatedTrajectory(Trajectory t1, Trajectory t2){
		this(Arrays.asList(t1,t2));
	}
	
	public ConcatenatedTrajectory(List<Trajectory> inputTrajectories){
		
		if(inputTrajectories.isEmpty()){
			throw new IllegalArgumentException("Empty list of trajectories!");
		}
		
		trajectories = new LinkedList<Trajectory>(inputTrajectories);
		
		minTime = trajectories.getFirst().getMinTime();
		maxTime = trajectories.getLast().getMaxTime();
		
		if(Double.isNaN(minTime) || Double.isNaN(maxTime)){
			throw new IllegalArgumentException("Trajectory minTime or maxTime must be a number! - minTime:"+minTime+", maxTime:"+maxTime);
		}
		
		//check consistency
		double t = minTime;
		
		for(Trajectory trajectory : trajectories){
			if(trajectory.getMinTime() != t){
				throw new IllegalArgumentException("Concatenated trajectories must align in time!");
			}
			
			t = trajectory.getMaxTime();
		}
		
		if(Double.isNaN(t)){
			throw new IllegalArgumentException("Trajectory maxTime must be a number! - t:"+t);
		}
		
		
	}

	@Override
	public double getMinTime() {
		return minTime;
	}

	@Override
	public double getMaxTime() {
		return maxTime;
	}

	@Override
	public OrientedPoint getPosition(double t) {
		if(Double.isNaN(t) || t < minTime || t > maxTime){
			LOGGER.warn("Time point out of range! t:"+t+" - minTime:"+minTime+", maxTime:"+maxTime);
			return null;
		}
		
		for(Trajectory trajectory : trajectories){
			if(t >= trajectory.getMinTime() && t <= trajectory.getMaxTime()){
				return trajectory.getPosition(t);
			}
		}
		
		LOGGER.warn("Time point not found! t:"+t+" - minTime:"+minTime+", maxTime:"+maxTime);
		
		throw new RuntimeException("Time point t:"+t+" not found! - minTime:"+minTime+", maxTime:"+maxTime);
		
	}

	@Override
	public String toString() {
		return trajectories.toString();
	}
}
