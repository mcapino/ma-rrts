package cz.agents.alite.trajectorytools.util;


public class WaypointTime {
    private static final long serialVersionUID = 1136064568843307511L;
    private static final double EPSILON = 1e-6;

    Waypoint waypoint;
    double time;

    public WaypointTime(Waypoint waypoint, double time) {
        super();
        this.waypoint = waypoint;
        this.time = time;
    }

    public Waypoint getWaypoint() {
        return waypoint;
    }

    public double getTime() {
        return time;
    }

    @Override
    public String toString() {
        return "(" + waypoint + "@" + String.format("%.2f", time) + ")";
    }




}
