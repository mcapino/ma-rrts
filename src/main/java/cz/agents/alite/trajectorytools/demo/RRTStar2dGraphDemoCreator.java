package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Random;

import javax.vecmath.Point2d;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.GraphPaths;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.TimeWeightedSpatialManeuverGraph;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatial.rrtstar.SpatialGraphDomain;
import cz.agents.alite.trajectorytools.planner.DistanceFunction;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphLayer.GraphProvider;
import cz.agents.alite.trajectorytools.vis.NodeIdLayer;
import cz.agents.alite.trajectorytools.vis.RRTStarLayer;
import cz.agents.alite.trajectorytools.vis.RegionsLayer;
import cz.agents.alite.trajectorytools.vis.RegionsLayer.RegionsProvider;
import cz.agents.alite.trajectorytools.vis.TrajectoryLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoryLayer.TrajectoryProvider;
import cz.agents.alite.trajectorytools.vis.projection.DefaultProjection;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;


public class RRTStar2dGraphDemoCreator implements Creator {

    double width = 1000;
    double height = 1000;
    int cols  = 30;
    int rows = 30;

    private Graph<Waypoint, SpatialManeuver> graph;
    RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstar;

    Waypoint start;
    Waypoint target;

    Collection<SpaceRegion> obstacles = new LinkedList<SpaceRegion>();

    Trajectory trajectory = null;

    Random random = new Random(1);
    BoxRegion bounds;

    @Override
    public void init(String[] args) {

    }

    @Override
    public void create() {

        graph = new TimeWeightedSpatialManeuverGraph<Waypoint, SpatialManeuver>(
                    SpatialGridFactory.create4WayGrid(width, height, cols, rows, 1.0));
        bounds = new BoxRegion(new SpatialPoint(0,0,0), new SpatialPoint(width, height, 1.0));

        createObstacles(75, 120);
        SpatialGraphs.cutOutObstacles(graph, obstacles);

        start  = SpatialGraphs.getNearestVertex(graph, new SpatialPoint(0,0,0));
        target = SpatialGraphs.getNearestVertex(graph, new SpatialPoint(900,900,0));

        DistanceFunction<Waypoint> distance = new DistanceFunction<Waypoint>() {

            @Override
            public double getDistance(Waypoint current, Waypoint goal) {
                return current.distance(goal);
            }
        };

        Domain<Waypoint, GraphPath<Waypoint, SpatialManeuver>> domain
            = new SpatialGraphDomain<Waypoint, SpatialManeuver>(graph, target, distance, 100, random);

        rrtstar = new RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>>(domain, start, 1300);

        createVisualization();

        int n=100000;
        for (int i=0; i<n; i++) {
            rrtstar.iterate();

            if (rrtstar.getBestVertex() != null) {
                //System.out.println("Best vertex: " + rrtstar.getBestVertex());
                GraphPath<Waypoint, GraphPath<Waypoint, SpatialManeuver>> pathPath = rrtstar.getBestPath();
                GraphPath<Waypoint, SpatialManeuver> path = GraphPaths.concatenate(graph, pathPath.getEdgeList());

                trajectory = new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(0.0, path, width+height);
                //trajectory =  new SampledTrajectory(trajectory, 100);
            }


            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

    private void createObstacles(int n, double maxSize) {

        for (int i=0; i<n; i++) {
            double size = random.nextDouble() * maxSize;
            double x = bounds .getCorner1().x + random.nextDouble() *  (bounds.getCorner2().x - bounds.getCorner1().x);
            double y = bounds.getCorner1().y + random.nextDouble() *  (bounds.getCorner2().y - bounds.getCorner1().y);
            SpaceRegion obstacle = new BoxRegion(new SpatialPoint(x, y, 0), new SpatialPoint(x+size,y+size,750));
            obstacles.add(obstacle);
        }
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 2000, 2000);
        VisManager.setSceneParam(new SceneParams(){

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(500, 500);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.5;
            } });
        VisManager.init();



        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // graph
        VisManager.registerLayer(GraphLayer.create( new GraphProvider<Waypoint, SpatialManeuver>() {

            @Override
            public Graph<Waypoint, SpatialManeuver> getGraph() {
                return graph;
            }
        }, Color.GRAY, Color.GRAY, 1, 4));

        // RRT
        VisManager.registerLayer(RRTStarLayer.create(rrtstar, new DefaultProjection<SpatialPoint>(),  Color.MAGENTA, Color.BLUE, 2, 8, true));

        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

            @Override
            public Collection<SpaceRegion> getSpaceRegions() {
                LinkedList<SpaceRegion> regions = new LinkedList<SpaceRegion>();
                regions.add(bounds);
                regions.addAll(obstacles);
                return regions;
            }

        }, new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, point.y);
            }
        }, Color.BLACK, 1));

        VisManager.registerLayer(NodeIdLayer.create(graph, Color.BLACK, 1, "n"));



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
        }, Color.RED, 5.0, 2000.0, 't'));


        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

  }
