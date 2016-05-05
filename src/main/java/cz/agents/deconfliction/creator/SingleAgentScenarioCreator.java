package cz.agents.deconfliction.creator;

import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Properties;
import java.util.Random;

import javax.vecmath.Point2d;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer.MissionsProvider;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatial.vis.SpatialGraphRRTStarLayer;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.simulation.SimulatedAgentEnvironment;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphLayer.GraphProvider;
import cz.agents.alite.trajectorytools.vis.NodeIdLayer;
import cz.agents.alite.trajectorytools.vis.RegionsLayer;
import cz.agents.alite.trajectorytools.vis.RegionsLayer.RegionsProvider;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer.TimeProvider;
import cz.agents.alite.trajectorytools.vis.SimulationControlLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer.TrajectoriesProvider;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.deconfliction.probleminstance.ToyWorldSingleAgentProblemInstance;
import cz.agents.deconfliction.solver.central.Counters;
import cz.agents.deconfliction.solver.central.Listener;
import cz.agents.deconfliction.solver.central.ODState;
import cz.agents.deconfliction.solver.central.SearchResult;
import cz.agents.deconfliction.solver.central.SingleAgentRRTStarSolver;
import cz.agents.deconfliction.solver.central.jaastar.JointActionAStarSolver;
import cz.agents.deconfliction.util.Trajectories;


public class SingleAgentScenarioCreator implements Creator {

    ////////////////////////////////////////////////////////////

    @Override
    public void create() {
        final int SIZE = 55;
        create(RRT, SIZE, SIZE, 0.15, 12, true);
    }

    ////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(SingleAgentScenarioCreator.class);

    private static final String JA = "ja";
    private static final String RRT = "graphrrt";

    private DirectedGraph<Waypoint, SpatialManeuver> graph;
    private Collection<SpaceRegion> obstacles;
    private BoxRegion bounds;
    private SpatialPoint target;
    private SpatialPoint start;
    private double maxTime;

    private Trajectory trajectory = null;
    private SimulatedAgentEnvironment simulation = null;
    private double agentSizeRadius;

    private RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrt;

    static class Result {

        final String alg;
        final EvaluatedTrajectory trajectory;
        final long runtimeMs;
        final long expandedStates;

        public Result(String alg,
                EvaluatedTrajectory result,
                long runtimeMs,
                long expandedStates) {
            super();
            this.alg = alg;
            this.trajectory = result;
            this.runtimeMs = runtimeMs;
            this.expandedStates = expandedStates;
        }

        public double getCost() {
            return trajectory.getCost();
        }

        public void print() {
            System.out.println("Algorithm: " + alg);
            System.out.println("Cost: " + getCost() + "");
            System.out.println("Runtime: " + runtimeMs + "ms");
            System.out.println("Expanded states: " + expandedStates + "");
        }

        public EvaluatedTrajectory getTrajectory() {
            return trajectory;
        }
    }

    class SolverListener implements Listener, SingleAgentRRTStarSolver.Listener{
        long runtimeStart;

        public SolverListener(long runtimeStart) {
            super();
            this.runtimeStart = runtimeStart;
        }

        @Override
        public void notifyNewSolution(EvaluatedTrajectory[] trajectories, boolean provedOptimal) {
            SingleAgentScenarioCreator.this.trajectory = Trajectories.toTrajectoryArray(trajectories)[0];
            LOGGER.info(String.format(">>> New solution found. Time: %d ms Cost: %.2f Exp. states: %d", (System.currentTimeMillis() - runtimeStart), Trajectories.evaluateCost(trajectories), Counters.statesExpanded));
            updateTrajectoryInSimulation();
        }

        @Override
        public void notifyNewSolution(EvaluatedTrajectory trajectory) {
             SingleAgentScenarioCreator.this.trajectory = trajectory;
             LOGGER.info(String.format(">>> New solution found. Time: %d ms Cost: %.2f Iterations: %d Exp. states: %d ", (System.currentTimeMillis() - runtimeStart), trajectory.getCost(), Counters.iterations, Counters.statesExpanded));
             updateTrajectoryInSimulation();

        }
    };


    @Override
    public void init(String[] args) {
        // Set-up the logger
        String overrideLogFileName = null;
        if (args.length>1) overrideLogFileName  = args[1];

        Properties prop = new Properties();
        try {
            prop.load(new FileInputStream("resources" + File.separator + "log4j" + File.separator + "log4j.properties"));
        } catch (Exception ex){
            ex.printStackTrace();
        }
        if (overrideLogFileName != null) {
            prop.setProperty("log4j.appender.R.File", overrideLogFileName);
        }
        PropertyConfigurator.configure(prop);
    }



    public void create(String alg, int cols, int rows, double obstaclesRatio, int seed, boolean showVis) {
        ToyWorldSingleAgentProblemInstance problem = null;
        problem = new ToyWorldSingleAgentProblemInstance(cols, rows, obstaclesRatio, 0.8, seed);

        graph = problem.getGraph();
        obstacles = problem.getObstacles();
        bounds = problem.getBounds3d();
        start = problem.getStart();
        target = problem.getTarget();
        agentSizeRadius = problem.getAgentSizeRadius()*2;
        maxTime = bounds.getXSize() + bounds.getYSize();

        if (showVis) {
            createSimulation();
            createVisualization();
        }

        Result result = null;
        if (alg.equals(JA)) {
            result = solveJA(graph, start, target, maxTime);
        } else if (alg.equals(RRT)) {
            result = solveRRT(graph, start, target, maxTime);
        }

        result.print();

        if (result.getTrajectory() != null) {
            trajectory = result.getTrajectory();
            updateTrajectoryInSimulation();
        }

    }

    private Result solveJA(
            DirectedGraph<Waypoint, SpatialManeuver> graph,
            SpatialPoint start, SpatialPoint target, double maxTime) {

        Waypoint[] startWaypoints = toWaypoints(graph, new SpatialPoint[]{start});
        Waypoint[] goalWaypoints = toWaypoints(graph, new SpatialPoint[]{target});

        long startTime = System.currentTimeMillis();

        JointActionAStarSolver solver = new JointActionAStarSolver(startWaypoints, goalWaypoints, graph, agentSizeRadius, maxTime, 1.0);
        SearchResult res = solver.solveTrajectories(Long.MAX_VALUE);

        long runtimeMs = System.currentTimeMillis() - startTime;
        return new Result(JA, res.getTrajectories()[0], runtimeMs, Counters.statesExpanded);
    }

    private Result solveRRT(
            DirectedGraph<Waypoint, SpatialManeuver> graph,
            SpatialPoint start, SpatialPoint target, double maxTime) {

        Waypoint startWaypoint = toWaypoint(graph, start);
        Waypoint goalWaypoint  = toWaypoint(graph, target);

        final long startTime = System.currentTimeMillis();

        double maxEdgeLength = 5;
        double gamma = 0.9*maxTime;
        double tryGoal = 0.1;

        SingleAgentRRTStarSolver solver = new SingleAgentRRTStarSolver(
                graph, bounds, startWaypoint, goalWaypoint, maxTime,
                gamma, maxEdgeLength, tryGoal, new Random(1));

        rrt = solver.getRRTStarPlanner();

        solver.registerListener(new SolverListener(System.currentTimeMillis()));
        EvaluatedTrajectory traj = solver.solve(3600000);

        long runtimeMs = System.currentTimeMillis() - startTime;
        return new Result(RRT, traj, runtimeMs, Counters.statesExpanded);
    }

    protected static ODState[] toAgentStates(DirectedGraph<Waypoint, SpatialManeuver> maneuvers, SpatialPoint[] spatialPoints) {
        ODState[] agentStates = new ODState[spatialPoints.length];

        for(int i=0; i < spatialPoints.length; i++) {
            Waypoint waypoint = SpatialGraphs.getNearestVertex(maneuvers, spatialPoints[i]);
            agentStates[i] = new ODState(waypoint, 0.0, null, null, 0);
        }

        return agentStates;
    }

    protected static Waypoint[] toWaypoints(DirectedGraph<Waypoint, SpatialManeuver> maneuvers, SpatialPoint[] spatialPoints) {
        Waypoint[] waypoints = new Waypoint[spatialPoints.length];

        for(int i=0; i < spatialPoints.length; i++) {
            waypoints[i] = SpatialGraphs.getNearestVertex(maneuvers, spatialPoints[i]);
        }

        return waypoints;
    }

    protected static Waypoint toWaypoint(DirectedGraph<Waypoint, SpatialManeuver> maneuvers, SpatialPoint spatialPoint) {
        return SpatialGraphs.getNearestVertex(maneuvers, spatialPoint);
    }

    private void createSimulation() {
        simulation  = new SimulatedAgentEnvironment();
    }

    private void updateTrajectoryInSimulation() {
        simulation.clearTrajectories();
        simulation.updateTrajectory("a", trajectory);
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768);
        VisManager.setSceneParam(new SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(bounds.getCorner2().x / 2, bounds.getCorner2().x / 2);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 10.0;
            }

        });
        VisManager.init();


        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        VisManager.registerLayer(GraphLayer.create(new GraphProvider<Waypoint, SpatialManeuver>() {

            @Override
            public Graph<Waypoint, SpatialManeuver> getGraph() {
                return graph;
            }}, Color.GRAY, Color.GRAY, 1, 4));


        VisManager.registerLayer(NodeIdLayer.create(graph, Color.GRAY, 1, "n"));



        // X-Y TOP View ////////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, point.y);
            }
        });




        // X-Z SIDE View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, -point.z + bounds.getYSize() + bounds.getZSize() + 2);
            }
        });

        // Y-Z SIDE View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.z + bounds.getXSize() + 2, point.y);
            }
        });

        // Y-T View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(-point.w - 2, point.y);
            }
        });


        // X-T View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, -point.w  - 2);
            }
        });

        ///////////////////////////////////////////////////////////////////////////

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());

        // Simulation Control
        VisManager.registerLayer(SimulationControlLayer.create(simulation));
    }

    private void createView(final ProjectionTo2d<TimePoint> projection) {


        VisManager.registerLayer(MissionsLayer.create(new MissionsProvider(){

            @Override
            public SpatialPoint[] getStarts() {
                return new SpatialPoint[]{start};
            }

            @Override
            public SpatialPoint[] getTargets() {
                return new SpatialPoint[]{target};
            }}, new ProjectionTo2d<SpatialPoint>(){

                @Override
                public Point2d project(SpatialPoint point) {
                    return projection.project(new TimePoint(point, 0.0));
                }}, Color.RED));

        VisManager.registerLayer(SpatialGraphRRTStarLayer.create(new SpatialGraphRRTStarLayer.RRTStarProvider<Waypoint, GraphPath<Waypoint, SpatialManeuver>>()  {

            @Override
            public RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> getRRTStar() {
                return rrt;
            }
        }, projection, 0.0, maxTime, 3));

        VisManager.registerLayer(TrajectoriesLayer.create(new TrajectoriesProvider() {

            @Override
            public Trajectory[] getTrajectories() {
                return new Trajectory[] {trajectory};
            }
        }, projection, 0.1, maxTime, 't'));

        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

            @Override
            public Collection<SpaceRegion> getSpaceRegions() {
                LinkedList<SpaceRegion> regions = new LinkedList<SpaceRegion>();
                regions.add(bounds);
                return regions;
            }
        },
        projection,
        Color.BLACK, 1));

        // Obstacles
//        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {
//
//            @Override
//            public Collection<SpaceRegion> getSpaceRegions() {
//                List<SpaceRegion> regions = new LinkedList<SpaceRegion>();
//                regions.addAll(obstacles);
//                return regions;
//            }
//
//
//        },
//        projection,
//        Color.BLACK, 2));

        VisManager.registerLayer(SimulatedCylindricAgentLayer.create(simulation.getAgentStorage(), projection, new TimeProvider() {

            @Override
            public double getTime() {
                return simulation.getTime();
            }
        }, agentSizeRadius/2, 0.1));


    }





  }
