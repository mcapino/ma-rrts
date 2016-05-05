package cz.agents.alite.trajectorytools.trajectory;

import java.util.ArrayList;
import java.util.List;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Vector;

public class WaypointStraightLineTrajectory implements Trajectory {

    private ArrayList<TimePoint> waypoints;

    public WaypointStraightLineTrajectory(List<TimePoint> waypoint) {
        assert(waypoint.size() > 0);
        
        if(waypoint.size() < 1){
        	throw new IllegalArgumentException("Empty list of waypoints is not a valid trajectory! " + waypoint);
        }
        
        this.waypoints = new ArrayList<TimePoint>(waypoint);
    }

    @Override
    public double getMinTime() {
        return waypoints.get(0).getTime();
    }

    @Override
    public double getMaxTime() {
        return waypoints.get(waypoints.size()-1).getTime();
    }

    @Override
    public OrientedPoint getPosition(double t) {
        assert(t >= getMinTime() && t <= getMaxTime());
        
        if(t == getMinTime() && waypoints.size() == 1){
        	TimePoint start = waypoints.get(0);
        	return new OrientedPoint(start.getSpatialPoint(), new Vector(0,1,0));
        }

        for (int i=0; i < waypoints.size()-1; i++) {
            TimePoint start = waypoints.get(i);
            TimePoint end = waypoints.get(i+1);
            if (t >= start.getTime() && t <= end.getTime()) {
                double alpha = (t - start.getTime()) / (end.getTime() - start.getTime());
                assert(alpha >= -0.00001 && alpha <= 1.00001);

                SpatialPoint pos = SpatialPoint.interpolate(start.getSpatialPoint(), end.getSpatialPoint(), alpha);
                Vector dir;
                if (!end.getSpatialPoint().equals(start.getSpatialPoint())) {
                    dir = Vector.subtract(end.getSpatialPoint(), start.getSpatialPoint());
                    dir.normalize();
                } else {
                    dir = new Vector(0,1,0);
                }

                return new OrientedPoint(pos, dir);
            }
        }

        throw new RuntimeException("Requesting position for time "+t+", which is undefined in this trajectory. tmin: " + waypoints.get(0) + ", tmax: " + waypoints.get(waypoints.size()-1));
    }
    
    
    public ArrayList<TimePoint> getWaypoints() {
		return waypoints;
	}

	public String toString(){
    	StringBuilder sb = new StringBuilder();
        sb.append("WPT(");

		for(TimePoint tp : waypoints){
			sb.append(tp);
		}
		
        sb.append(" )");
        return sb.toString();
    }

}
