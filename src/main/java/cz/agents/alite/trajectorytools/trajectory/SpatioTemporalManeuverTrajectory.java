package cz.agents.alite.trajectorytools.trajectory;

import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.maneuvers.SpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
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

public class SpatioTemporalManeuverTrajectory<V extends TimePoint, E extends SpatioTemporalManeuver> implements Trajectory {
    static Logger LOGGER = Logger.getLogger(SpatioTemporalManeuverTrajectory.class);

    private List<E> maneuvers = null;

    private TimePoint startPoint;
    private TimePoint endPoint;

    Graph<V,E> graph;

    private double duration = Double.POSITIVE_INFINITY;

    public SpatioTemporalManeuverTrajectory(GraphPath<V,E> graphPath, double duration) {
        this.startPoint = graphPath.getStartVertex();
        this.endPoint = graphPath.getEndVertex();
        this.maneuvers = graphPath.getEdgeList();

        this.duration = duration;
        this.graph = graphPath.getGraph();
    }

    @Override
    public OrientedPoint getPosition(double t) {

        TimePoint currentPoint = startPoint;
        Vector currentDirection = new Vector(1,0,0);

        if (t < startPoint.getTime() && t > endPoint.getTime()) {
            return null;
        }


        if (maneuvers != null)  {
            for (E maneuver: maneuvers) {
                TimePoint nextWaypoint = graph.getEdgeTarget(maneuver);

                if ( maneuver.getStartTime() <= t && t <= maneuver.getStartTime() + maneuver.getDuration()) {
                    // linear approximation
                    OrientedPoint pos = maneuver.getTrajectory().getPosition(t);
                    //LOGGER.info("Asked for t: "+t+" returned:" + pos);
                    return pos;
                }
                currentPoint = nextWaypoint;
            }
        }
        if (t >= endPoint.getTime()) {
            return new OrientedPoint(currentPoint.getSpatialPoint(), currentDirection);
        }

        return null;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SpatioTemporalManeuverTrajectory){
            SpatioTemporalManeuverTrajectory other = (SpatioTemporalManeuverTrajectory) obj;
            if (startPoint.equals(other.startPoint) &&
                    endPoint.equals(other.endPoint) &&
                    maneuvers.equals(other.maneuvers)) {
                return true;
            }
        }

        return false;
    }

    @Override
    public int hashCode() {
        int hashCode = startPoint.hashCode();

        for (SpatioTemporalManeuver edge: maneuvers) {
            hashCode = hashCode ^ edge.hashCode();
        }

        return hashCode;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MT(");


        if (!maneuvers.isEmpty()) {
            sb.append("" + graph.getEdgeSource(maneuvers.get(0)));
        }

        for (E maneuver: maneuvers) {
            sb.append(" " + graph.getEdgeTarget(maneuver));
        }
        sb.append(" )");
        return sb.toString();
    }

    @Override
    public double getMinTime() {
        return startPoint.getTime();
    }

    @Override
    public double getMaxTime() {
        return startPoint.getTime() + duration;
    }
}
