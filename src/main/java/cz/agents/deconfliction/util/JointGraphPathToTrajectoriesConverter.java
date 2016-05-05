package cz.agents.deconfliction.util;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectoryWrapper;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class JointGraphPathToTrajectoriesConverter {

    public static EvaluatedTrajectory[] convert(GraphPath<JointWaypointState, JointGraphPath> path, Graph<Waypoint, SpatialManeuver>[] maneuverGraphs, double maxTime) {

        JointWaypointState startJointState = path.getStartVertex();
        List<JointGraphPath> jointManeuvers = path.getEdgeList();
        Graph<JointWaypointState, JointGraphPath> jointGraph = path.getGraph();

        int nAgents = path.getStartVertex().nAgents();
        EvaluatedTrajectory[] trajectories = new EvaluatedTrajectory[nAgents];

        for (int i = 0; i < nAgents; i++) {
            // Processing agent i
            Graph<Waypoint, SpatialManeuver> graph = maneuverGraphs[i];

            Waypoint startWaypoint = startJointState.getPosition(i);
            List<SpatialManeuver> edges = new LinkedList<SpatialManeuver>();
            Waypoint endWaypoint = path.getEndVertex().getPosition(i);
            double cost = 0.0;

            for (JointGraphPath jointManeuver : jointManeuvers) {
                GraphPath<Waypoint, SpatialManeuver> graphPath = jointManeuver.getPath(i);
                edges.addAll(graphPath.getEdgeList());
                cost += graphPath.getWeight();
            }

            GraphPath<Waypoint, SpatialManeuver> singleGraphPath
                = new GraphPathImpl<Waypoint, SpatialManeuver>(graph, startWaypoint, endWaypoint, edges, cost);

            trajectories[i] = new EvaluatedTrajectoryWrapper(new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(0, singleGraphPath, maxTime), cost);
        }

        return trajectories;
    }

}
