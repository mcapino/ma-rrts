package cz.agents.alite.trajectorytools.predictive;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class LinearPredictor implements PredictiveTrajectory.PredictorInterface {
	
	private OrientedPoint lo = null;
	private double lt = Double.NEGATIVE_INFINITY;
	private OrientedPoint co = null;
	private double ct = Double.NEGATIVE_INFINITY;

	@Override
	public void addObservation(OrientedPoint pos, double time) {
		lo = co;
		co = pos;
		lt = ct;
		ct = time;
	}

	@Override
	public OrientedPoint predict(double time) {
		if(co==null){
			return new OrientedPoint(0,0, 0, 0, 0, 0);
		}
		
		if(lo==null){
			return co;
		}
		
		if(ct == lt){
			return co;
		}
		
		SpatialPoint p = (SpatialPoint) lo.clone();
		double a = (time-lt)/(ct-lt);
		p.interpolate(co, a);
		
		return new OrientedPoint(p,co.orientation);
	}

	@Override
	public double getMinTime() {
		return lt;
	}

}
