package cz.agents.alite.trajectorytools.trajectory;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Vector;


/**
 * A sampling-based trajectory approximation.
 */
public class SampledTrajectory implements Trajectory{
    
    double minTime;
    double maxTime;
    
    private double samplingInterval; // In seconds
    OrientedPoint[] samples;
    
    public SampledTrajectory(Trajectory continuousTrajectory, double samplingInterval) {

        if (samplingInterval <= 0) {
            throw new RuntimeException("Sampling interval must be >= 0.");
        }
        
        if (continuousTrajectory.getMaxTime() == Double.POSITIVE_INFINITY) {
            throw new RuntimeException("Cannot create a sampled approximation of an infinite-length trajectory.");
        }

        this.samplingInterval = samplingInterval;
        this.maxTime = continuousTrajectory.getMaxTime();
        this.minTime = continuousTrajectory.getMinTime();
        
        int nSamples = (int) Math.floor((maxTime - minTime) / samplingInterval);
        
        samples = new OrientedPoint[nSamples+1];
        
        for (int i=0; i <= nSamples; i++) {
            samples[i] = continuousTrajectory.getPosition(minTime + i * samplingInterval);
        }
    }

    /**
     * @return time interval between two samples in seconds
     */
    public double getSamplingInterval() {
        return samplingInterval;
    }

    @Override
    public OrientedPoint getPosition(double t) {
    	int leftSampleNo = (int) Math.floor( (t - minTime) / samplingInterval );
    	int rightSampleNo = (int) Math.ceil( (t - minTime) / samplingInterval );
    	
    	if (leftSampleNo == rightSampleNo) {
    		return samples[leftSampleNo];
    	} else {
    		if (rightSampleNo < samples.length) {
    			// linear approx.
				
				double alpha =  (t - leftSampleNo * samplingInterval) / samplingInterval;
				assert(alpha >= 0.0 && alpha <= 1.0);
				
				OrientedPoint left = samples[leftSampleNo];
				OrientedPoint right = samples[rightSampleNo];
						
				SpatialPoint pos = SpatialPoint.interpolate(left, right, alpha);
				Vector dir;
				if (!left.equals(right)) {
					dir = Vector.subtract(right, left);
					dir.normalize();
				} else {
					dir = new Vector(0,1,0);
				}
				
				return new OrientedPoint(pos, dir);
    		} else {
    			// assume we are on the last sample
    			return samples[leftSampleNo];
    		}
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
}
