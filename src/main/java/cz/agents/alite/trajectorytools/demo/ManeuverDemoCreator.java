package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;

import javax.vecmath.Point2d;

import org.jgrapht.Graph;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PathPlanner;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.trajectory.SampledTrajectory;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphLayer.GraphProvider;
import cz.agents.alite.trajectorytools.vis.TrajectoryLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoryLayer.TrajectoryProvider;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;


public class ManeuverDemoCreator implements Creator {

    private Graph<Waypoint, SpatialManeuver> graph;
    private Trajectory trajectory;
    private PlannedPath<Waypoint, SpatialManeuver> plannedPath;

    @Override
    public void init(String[] args) {

    }

    @Override
    public void create() {
        graph = SpatialGridFactory.create4WayGrid(10, 10, 10, 10, 1.0);

        replan();
        trajectory =  new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(0.0, plannedPath, plannedPath.getWeight());
        trajectory =  new SampledTrajectory(trajectory, 0.9);
        createVisualization();
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 20, 20);
        VisManager.setSceneParam(new SceneParams(){

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(5,5);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 50;
            } } );
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // graph
        VisManager.registerLayer(GraphLayer.create(new GraphProvider<Waypoint, SpatialManeuver>() {

            @Override
            public Graph<Waypoint, SpatialManeuver> getGraph() {
                return graph;
            }
        }, Color.GRAY, Color.GRAY, 1, 4));

        // draw the shortest path
        //VisManager.registerLayer(GraphPathLayer.create(graph, path, Color.RED, Color.RED.darker(), 2, 4));

        VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryProvider() {

            @Override
            public Trajectory getTrajectory() {
                return trajectory;
            }
        }, new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, point.y);
            }
        }, Color.BLUE, 0.1, 100.0, 't'));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {

        try {
            PathPlanner<Waypoint, SpatialManeuver> aStar
                = new AStarPlanner<Waypoint, SpatialManeuver>();

            aStar.setHeuristicFunction(new HeuristicFunction<Waypoint>() {
            @Override
                public double getHeuristicEstimate(Waypoint current, Waypoint goal) {
                    return current.distance(goal);
                }
            });

            plannedPath = aStar.planPath(
                    graph,
                    SpatialGraphs.getNearestVertex(graph, new SpatialPoint(0, 0, 0)),
                    SpatialGraphs.getNearestVertex(graph, new SpatialPoint(10, 10, 0))
                    );
        } catch (Exception e) {
            System.out.println("Error: " + e.getMessage());
            e.printStackTrace();
            plannedPath = null;
        }
    }
}
