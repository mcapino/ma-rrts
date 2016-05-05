package cz.agents.alite.trajectorytools.predictive;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class PredictiveTrajectory implements Trajectory {
	
	PredictorInterface predictor;
	final double maxPredictionTime;

	public PredictiveTrajectory(PredictorInterface predictor){
		this(predictor,Double.POSITIVE_INFINITY);
	}
	
	public PredictiveTrajectory(PredictorInterface predictor,double maxPredictionTime){
		this.predictor = predictor;
		this.maxPredictionTime = maxPredictionTime;
	}
	
	@Override
	public double getMinTime() {
		return predictor.getMinTime();
	}

	@Override
	public double getMaxTime() {
		return predictor.getMinTime() + maxPredictionTime;
	}

	@Override
	public OrientedPoint getPosition(double t) {
		return predictor.predict(t);
	}
	
	
	
	public interface PredictorInterface{
		
		public void addObservation(OrientedPoint pos, double time);
		
		public OrientedPoint predict(double time);
		
		public double getMinTime();
		
	}
	
}
