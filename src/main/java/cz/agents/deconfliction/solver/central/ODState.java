package cz.agents.deconfliction.solver.central;

import java.text.DecimalFormat;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class ODState {
    Waypoint currentWaypoint;
    double time;
    SpatialManeuver nextManeuver;
    Waypoint nextWaypoint;
    double cost;
    
    public ODState(Waypoint currentWaypoint, double time, SpatialManeuver nextManeuver, Waypoint nextWaypoint, double cost) {
        super();
        this.time = time;
        this.currentWaypoint = currentWaypoint;
        this.nextManeuver = nextManeuver;
        this.nextWaypoint = nextWaypoint;
        this.cost = cost;
    }

    public Waypoint getCurrentWaypoint() {
        return currentWaypoint;
    }

    public SpatialManeuver getNextManeuver() {
        return nextManeuver;
    }

    public Waypoint getNextWaypoint() {
        return nextWaypoint;
    }

    public double getTime() {
        return time;
    }

    public double getCost() {
        return cost;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof ODState) {
            ODState other = (ODState) obj;
            if (this.nextManeuver != null) {
                return this.currentWaypoint.equals(other.currentWaypoint) && this.nextManeuver.equals(other.nextManeuver) && this.time == other.time;
            } else {
                return this.currentWaypoint.equals(other.currentWaypoint) && this.time == other.time;
            }
        } else {
            return false;
        }
    }

    public boolean equalsIgnoreTime(Object obj) {
        if (obj instanceof ODState) {
            ODState other = (ODState) obj;
            if (this.nextManeuver != null) {
                return this.currentWaypoint.equals(other.currentWaypoint) && this.nextManeuver.equals(other.nextManeuver);
            } else {
                return this.currentWaypoint.equals(other.currentWaypoint);
            }
        } else {
            return false;
        }
    }

    @Override
    public String toString() {
        return "("+currentWaypoint+", "+nextManeuver+", " + nextWaypoint + ", t:"+ (new DecimalFormat("#.##")).format(time)+", c:" + (new DecimalFormat("#.##")).format(cost)+ ") ";
    }
    
    static ODState[] fromWaypoints(Waypoint[] positions) {
    	ODState[] states = new ODState[positions.length];
    	for (int i=0; i<positions.length; i++) {
    		if (positions[i] != null) {
    			states[i] = new ODState(positions[i], 0, null, null, 0);
    		}
    	}
    	
    	return states;
    }

}
