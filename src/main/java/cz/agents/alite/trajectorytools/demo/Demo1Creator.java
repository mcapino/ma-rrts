package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;
import java.awt.Rectangle;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView.ChangeListener;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PathPlanner;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer.PathProvider;
import cz.agents.alite.trajectorytools.vis.projection.DefaultProjection;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;


public class Demo1Creator implements Creator {

    private ObstacleGraphView graph;
    private PlannedPath<SpatialPoint, DefaultWeightedEdge> plannedPath;

    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        Graph<Waypoint, SpatialManeuver> listenableGraph = SpatialGridFactory.create4WayGrid(10, 10, 10, 10, 1.0);

        graph = ObstacleGraphView.createFromGraph(listenableGraph, new ChangeListener() {
            @Override
            public void graphChanged() {
                replan();
            }
        } );

        createVisualization();
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 20, 20);
        VisManager.setPanningBounds(new Rectangle(-500, -500, 1600, 1600));
        VisManager.init();

        Vis.setPosition(50, 50, 1);

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // graph with obstacles
        graph.createVisualization();

        // draw the shortest path
        VisManager.registerLayer(GraphPathLayer.create(new PathProvider<SpatialPoint, DefaultWeightedEdge>() {

            @Override
            public PlannedPath<SpatialPoint, DefaultWeightedEdge> getPath() {
                return plannedPath;
            }  }, new DefaultProjection<SpatialPoint>(), Color.RED, Color.RED.darker(), 2, 4));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {

        try {
            PathPlanner<SpatialPoint, DefaultWeightedEdge> aStar = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();

            aStar.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
            @Override
                public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
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
