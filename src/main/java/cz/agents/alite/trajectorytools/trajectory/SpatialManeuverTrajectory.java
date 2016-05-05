package cz.agents.alite.trajectorytools.trajectory;

import java.text.DecimalFormat;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Vector;

/**
 * A wrapper that interprets a graph path as a trajectory.
 * Start time must be be given. Then, the trajectory parameters should
 * be interpreted as follows:
 *
 *                      duration
 *       | --------------------------------> |
 *  start time                             max time
 *
 */

public class SpatialManeuverTrajectory<V extends SpatialPoint, E extends SpatialManeuver> implements EvaluatedTrajectory {

    private List<E> maneuvers = null;

    private SpatialPoint startWaypoint;
    private SpatialPoint endWaypoint;

    Graph<V,E> graph;

    private double startTime;
    private double duration = Double.POSITIVE_INFINITY;
    private double cost;


    public SpatialManeuverTrajectory(double startTime, GraphPath<V,E> graphPath, double duration) {
        this.startWaypoint = graphPath.getStartVertex();
        this.endWaypoint = graphPath.getEndVertex();
        this.maneuvers = graphPath.getEdgeList();

        this.startTime = startTime;
        this.duration = duration;
        this.graph = graphPath.getGraph();
        this.cost = graphPath.getWeight();
    }

    @Override
    public OrientedPoint getPosition(double t) {
        SpatialPoint currentWaypoint = startWaypoint;
        double currentWaypointTime = startTime;
        Vector currentDirection = new Vector(1,0,0);

        if (t < startTime && t > startTime + duration) {
            return null;
        }


        if (maneuvers != null)  {
            for (E maneuver: maneuvers) {
                SpatialPoint nextWaypoint = graph.getEdgeTarget(maneuver);
                double duration  = maneuver.getDuration();
                double nextWaypointTime = currentWaypointTime + duration;


                if ( currentWaypointTime <= t && t <= nextWaypointTime) {
                    // linear approximation
                    OrientedPoint pos = maneuver.getTrajectory(currentWaypointTime).getPosition(t);
                    return pos;
                }
                currentWaypoint = nextWaypoint;
                currentWaypointTime = nextWaypointTime;
            }
        }
        if (t >= currentWaypointTime) {
                   return new OrientedPoint(currentWaypoint, currentDirection);
        }

        return null;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SpatialManeuverTrajectory){
            SpatialManeuverTrajectory other = (SpatialManeuverTrajectory) obj;
            if (startWaypoint.equals(other.startWaypoint) &&
                    endWaypoint.equals(other.endWaypoint) &&
                    maneuvers.equals(other.maneuvers) &&
                    Math.abs(startTime - other.startTime) < 0.0001 &&
                    Math.abs(cost - other.cost) < 0.0001
                    ) {
                return true;
            }
        }

        return false;
    }

    @Override
    public int hashCode() {
        int hashCode = startWaypoint.hashCode();

        for (SpatialManeuver edge: maneuvers) {
            hashCode = hashCode ^ edge.hashCode();
        }

        return hashCode;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MT(@");
        DecimalFormat f = new DecimalFormat("#0.00");
        sb.append(f.format(startTime));

        if (!maneuvers.isEmpty()) {
            sb.append(" " + graph.getEdgeSource(maneuvers.get(0)));
        }

        for (E maneuver: maneuvers) {
            sb.append(" " + graph.getEdgeTarget(maneuver));
        }
        sb.append(" )");
        return sb.toString();
    }

    @Override
    public double getMinTime() {
        return startTime;
    }

    @Override
    public double getMaxTime() {
        return startTime + duration;
    }

    @Override
    public double getCost() {
        return cost;
    }


}
