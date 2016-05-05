package cz.agents.alite.trajectorytools.graph.spatiotemporal.region;

import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;

public class PolygonRegion implements SpaceTimeRegion, SpaceRegion {

	private static final double SMALL_NUM = 1e-6;
	private final SpatialPoint [] points;

	/**
	 * @param points Points have to be in a plane - otherwise the result is unpredictable
	 */
	public PolygonRegion(SpatialPoint [] points) {
		super();
		this.points = points;
	}

	static private boolean isInPolygon(SpatialPoint[] points, Point3d point, Projection projection) {
		int cn = 0;    // the crossing number counter

		// loop through all edges of the polygon
		for (int i=0; i<points.length; i++) {    // edge from V[i] to V[i+1]
			SpatialPoint currentPoint = points[i];
			SpatialPoint nextPoint = points[(i+1) % points.length];

			double cX = projection.getX(currentPoint); 
			double cY = projection.getY(currentPoint); 

			double nX = projection.getX(nextPoint); 
			double nY = projection.getY(nextPoint); 

			double pX = projection.getX(point); 
			double pY = projection.getY(point); 

			if (((cY <= pY) && (nY > pY))    // an upward crossing
					|| ((cY > pY) && (nY <= pY))) { // a downward crossing
				// compute the actual edge-ray intersect x-coordinate
				double vt = (pY - cY) / (nY - cY);
				if (pX < cX + vt * (nX - cX)) // P.x < intersect
					++cn;   // a valid crossing of y=P.y right of P.x
			}
		}
		return (cn % 2) == 1;    // 0 if even (out), and 1 if odd (in)
	}		

	@Override
	public boolean intersectsLine(TimePoint p1, TimePoint p2) {
		return intersectsLine(p1.getSpatialPoint(), p2.getSpatialPoint());
	}


	@Override
	public boolean intersectsLine(SpatialPoint p1, SpatialPoint p2) {
		Vector3d normal = new Vector3d();
		Point3d planeIntersection = getPlaneIntersection(points, p1, p2, normal);
		if (planeIntersection == null) {
			return false;
		}

		// project to 2D:
		// simply ignore one of the 3D coordinates and uses the other two. 
		// To optimally select the coordinate to ignore, compute a normal vector to the plane, 
		// and select the coordinate with the largest absolute value. 
		Projection projection;

		normal.absolute();
		
		if (normal.x > normal.y && normal.x > normal.z) {
			projection = new ProjectionYZ();
		} else if (normal.y > normal.x && normal.y > normal.z) {
			projection = new ProjectionXZ();
		} else {
			projection = new ProjectionXY();
		}

		return isInPolygon(points, planeIntersection, projection);
	}

	@Override
	public boolean isInside(SpatialPoint p) {
		// TODO Auto-generated method stub
		return false;
	}

	private static Point3d getPlaneIntersection(SpatialPoint[] points, SpatialPoint p0, SpatialPoint p1, Vector3d outNormal) {
		Vector3d u = new Vector3d(points[1]);
		u.sub(points[0]);
		Vector3d v = new Vector3d(points[2]); 
		v.sub(points[0]);

		//    	, v, n;             // triangle vectors
		//    	  Vector    dir, w0, w;          // ray vectors
		//    	  float     r, a, b;             // params to calc ray-plane intersect

		outNormal.cross(u, v);             // cross product
		if (outNormal.length() == 0) {           // triangle is degenerate
			return null;                 // do not deal with this case
		}

		Vector3d dir = new Vector3d(p1);
		dir.sub(p0);             
		Vector3d w0 =  new Vector3d(p0);;
		w0.sub(points[0]);
		double a = -outNormal.dot(w0);
		double b= outNormal.dot(dir);
		if (Math.abs(b) < SMALL_NUM) {     // ray is parallel to triangle plane
			if (a == 0) {               // ray lies in triangle plane
				return p0;
			} else { 
				return null;             // ray disjoint from plane
			}
		}

		// get intersect point of ray with triangle plane
		double r = a / b;
		if (r < 0.0 || r > 1.0) {                   // ray goes away from triangle
			return null;                  // => no intersect
		}
		Point3d intersection = new Point3d(p0);
		dir.scale(r);
		intersection.add(dir);

		return intersection;
	}

	@Override
	public boolean isInside(TimePoint p) {
		// no chance to aim exactly to the plane 
		// at least not when randomly selecting points in 3D
		return false;
	}

	public SpatialPoint [] getPoints() {
		return points;
	}

	@Override
	public String toString() {
		return "Poly"+Arrays.toString( points );
	}
		
	interface Projection {
		double getX(Point3d point);
		double getY(Point3d point);
	}

	class ProjectionXY implements Projection{
		@Override
		public double getX(Point3d point) {
			return point.x;
		}
		@Override
		public double getY(Point3d point) {
			return point.y;
		}
		
		@Override
		public String toString() {
			return "Projection XY";
		}
	}

	class ProjectionXZ implements Projection{
		@Override
		public double getX(Point3d point) {
			return point.x;
		}
		@Override
		public double getY(Point3d point) {
			return point.z;
		}

		@Override
		public String toString() {
			return "Projection XZ";
		}
	}

	class ProjectionYZ implements Projection{
		@Override
		public double getX(Point3d point) {
			return point.z;
		}
		@Override
		public double getY(Point3d point) {
			return point.y;
		}
		@Override
		public String toString() {
			return "Projection YZ";
		}
	}
}
