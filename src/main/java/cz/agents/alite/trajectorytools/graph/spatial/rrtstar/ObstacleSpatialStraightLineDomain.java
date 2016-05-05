package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.Collection;
import java.util.Iterator;
import java.util.Random;
import java.util.logging.Logger;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SphereRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.PolygonRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ObstacleSpatialStraightLineDomain extends SpatialStraightLineDomain {

    private double defaultZ;
    private SpatialPoint targetPoint;
    private double sampleTargetProbability;
    private Random random = new Random(1);
	private double sampleObstacleProbability;
	private int initRandomPoints;


	public ObstacleSpatialStraightLineDomain(BoxRegion bounds,
			Collection<SpaceRegion> obstacles, SpaceRegion targetRegion,
			SpatialPoint targetPoint, double speed, double defaultZ,
			double sampleTargetProbability, double sampleObstacleProbability,
			int initRandomPoints) {
		super(bounds, obstacles, targetRegion, speed);
		this.targetPoint = targetPoint;
		this.defaultZ = defaultZ;
		this.sampleTargetProbability = sampleTargetProbability;
		this.sampleObstacleProbability = sampleObstacleProbability;
		this.initRandomPoints = initRandomPoints;
	}

    @Override
    public SpatialPoint sampleState() {
        SpatialPoint point;
        if (initRandomPoints > 0) {
        	initRandomPoints--;
        	point = generateRandomPoint();
        } else if (random.nextDouble() <= sampleTargetProbability) {
            point = targetPoint;
        } else if ( !obstacles.isEmpty() && (random.nextDouble() <= sampleObstacleProbability) ) {
        	int obstNum = random.nextInt(obstacles.size());
        	Iterator<SpaceRegion> obstIt = obstacles.iterator();
        	for (int i=0; i<obstNum; i++) {
        		obstIt.next();
        	}
        	SpaceRegion obstReg = obstIt.next();
        	if (obstReg instanceof SphereRegion) {
        		SphereRegion obstacle = (SphereRegion) obstReg;
        		Point3d center = obstacle.getCenter();
                do {
	        		double radius = obstacle.getRadius() + obstacle.getRadius() * Math.abs( random.nextGaussian() ) / 5;
	        		double direction = random.nextDouble() * 2 * Math.PI;
	        		point = new SpatialPoint(
	        						center.x + radius * Math.cos(direction),
	        						center.y + radius * Math.sin(direction),
	        						defaultZ
	        						);
                } while (!isInFreeSpace(point));
        	} else if (obstReg instanceof PolygonRegion) {
        		PolygonRegion obstacle = (PolygonRegion) obstReg;
        		double length = 0;
        		SpatialPoint[] points = obstacle.getPoints();
        		for (int i = 0; i<points.length; i++) {
        			length += points[i].distance(points[(i+1) % points.length]);
        		}

        		point = null;
                do {
	        		double randomLength = random.nextDouble() * length;
	        		for (int i = 0; i<points.length; i++) {
	        			SpatialPoint nextPoint = points[(i+1) % points.length];
						double curLength = points[i].distance(nextPoint);
	        			if (randomLength < curLength) {
	        				SpatialPoint basePoint = SpatialPoint.interpolate(points[i], nextPoint, curLength/randomLength);  
	    	        		point = new SpatialPoint(
	    	        						basePoint.x + 20 * random.nextGaussian(),
	    	        						basePoint.y + 20 * random.nextGaussian(),
	    	        						defaultZ
	    	        						);
	        				break;
	        			} else {
	        				randomLength -= curLength;
	        			}
	        		}
                } while (!isInFreeSpace(point));
        	} else {
        		Logger.getAnonymousLogger().severe("Unknown obstacle type: " + obstReg.getClass().getName());
        		point = null;
        	}      	
        } else {
        	point = generateRandomPoint();
        }
        return point;
    }

	public SpatialPoint generateRandomPoint() {
		SpatialPoint point;
		do {
		    double x = bounds.getCorner1().x + (random.nextDouble() * (bounds.getCorner2().x - bounds.getCorner1().x));
		    double y = bounds.getCorner1().y + (random.nextDouble() * (bounds.getCorner2().y - bounds.getCorner1().y));
		    point = new SpatialPoint(x, y, defaultZ);
		} while (!isInFreeSpace(point));
		return point;
	}

    @Override
    public double estimateCostToGo(SpatialPoint p1) {
        return p1.distance(targetPoint);
    }


}
