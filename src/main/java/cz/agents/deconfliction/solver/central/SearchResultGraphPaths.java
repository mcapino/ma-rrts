package cz.agents.deconfliction.solver.central;

import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class SearchResultGraphPaths {

    public boolean finished;
    public GraphPath<Waypoint, SpatialManeuver>[] paths;

    public SearchResultGraphPaths(boolean finished,
            GraphPath<Waypoint, SpatialManeuver>[] paths) {
        super();
        this.finished = finished;
        this.paths = paths;
    }

    public GraphPath<Waypoint, SpatialManeuver>[] getPaths() {
        return paths;
    }

    public boolean isFinished() {
        return finished;
    }

    public boolean foundSolution() {
        return paths != null;
    }

}
