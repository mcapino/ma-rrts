package cz.agents.alite.trajectorytools.trajectory;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class TimeOffsetTrajectory implements Trajectory {
	
	private final Trajectory trajectory;
	private final double timeOffset;
	
	public TimeOffsetTrajectory(Trajectory trajectory, double timeOffset){
		this.trajectory = trajectory;
		this.timeOffset = timeOffset;
	}

	@Override
	public double getMinTime() {
		return trajectory.getMinTime() + timeOffset;
	}

	@Override
	public double getMaxTime() {
		return trajectory.getMaxTime() + timeOffset;
	}

	@Override
	public OrientedPoint getPosition(double t) {
		double to = t + timeOffset;
		if(to < trajectory.getMinTime())to = trajectory.getMinTime();
		if(to > trajectory.getMaxTime())to = trajectory.getMaxTime();
		return trajectory.getPosition(to);
	}
	
	public String toString(){
    	StringBuilder sb = new StringBuilder();
        sb.append("TO(");

		sb.append(trajectory);
		sb.append(", timeOffset:" + timeOffset);
		
        sb.append(" )");
        return sb.toString();
    }

}
