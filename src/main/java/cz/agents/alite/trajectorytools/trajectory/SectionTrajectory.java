package cz.agents.alite.trajectorytools.trajectory;

import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class SectionTrajectory implements Trajectory {
	
	private final Trajectory trajectory;
	private final double minTime;
	private final double maxTime;
	
	
	public SectionTrajectory(double startTime, Trajectory trajectory, double duration){
		this(trajectory,startTime,startTime + duration);
	}
	
	public SectionTrajectory(Trajectory trajectory, double maxTime){
		this(trajectory,trajectory.getMinTime(),maxTime);
	}
	
	public SectionTrajectory(double minTime, Trajectory trajectory){
		this(trajectory,minTime,trajectory.getMaxTime());
	}
	
	public SectionTrajectory(Trajectory trajectory, double minTime, double maxTime){
		this.trajectory = trajectory;
		this.minTime = minTime;
		this.maxTime = maxTime;
		
		if(minTime < trajectory.getMinTime() || maxTime > trajectory.getMaxTime()){
			throw new RuntimeException("Trajectory with minTime: " + trajectory.getMinTime() + ", maxTime: " + trajectory.getMaxTime() + " cannot be sectioned to minTime: " + minTime + ", maxTime: " + maxTime);
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
		
		if(t < minTime || t > maxTime){
			throw new RuntimeException("Requesting position for time "+t+", which is undefined in this trajectory. tmin: " + minTime + ", tmax: " + maxTime);
		}
		
		return trajectory.getPosition(t);
	}

}
