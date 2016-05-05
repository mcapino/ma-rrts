package cz.agents.deconfliction.creator;

import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Properties;

import javax.vecmath.Point2d;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.jointspatial.vis.JointOnGraphRRTStarLayer;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer.MissionsProvider;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatial.vis.SingleAgentSolversLayer;
import cz.agents.alite.trajectorytools.graph.spatial.vis.SingleAgentSolversLayer.SingleAgentSolversProvider;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
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
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer.TrajectoriesProvider;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.deconfliction.probleminstance.ToyWorldMultiAgentProblemInstance;
import cz.agents.deconfliction.probleminstance.ToyWorldRandomProblemInstance;
import cz.agents.deconfliction.probleminstance.ToyWorldSuperconflictProblemInstance;
import cz.agents.deconfliction.solver.central.BiasedMultiAgentRRTStarSolver;
import cz.agents.deconfliction.solver.central.Counters;
import cz.agents.deconfliction.solver.central.IDSolver;
import cz.agents.deconfliction.solver.central.Listener;
import cz.agents.deconfliction.solver.central.MultiAgentRRTStarSolver;
import cz.agents.deconfliction.solver.central.OASolver;
import cz.agents.deconfliction.solver.central.ODSolver;
import cz.agents.deconfliction.solver.central.ODState;
import cz.agents.deconfliction.solver.central.SearchResult;
import cz.agents.deconfliction.solver.central.SingleAgentRRTStarSolver;
import cz.agents.deconfliction.solver.central.jaastar.JointActionAStarSolver;
import cz.agents.deconfliction.util.Trajectories;


public class MultiAgentScenarioCreator implements Creator {

    public static void main(String[] args) {
        MultiAgentScenarioCreator creator = new MultiAgentScenarioCreator();
        creator.init(args);
        creator.create();
    }

    ////////////////////////////////////////////////////////////////////////

    @Override
    public void create() {
        if (args.length > 1) {
            createFromArgs();
        } else {

            final int SIZE = 20;

            create("default", BMARRT, RANDOM, SIZE, SIZE, (int)(3*SIZE), 10, 0.8, 0.1, 0.8, 5513, true, 5000);
        }
    }

    ///////////////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(MultiAgentScenarioCreator.class);

    private static final String SUPERCONFLICT = "superconflict";
    private static final String RANDOM = "random";

    private static final String JA = "ja";
    private static final String OD = "od";
    private static final String ODID = "odid";
    private static final String OA = "oa";
    private static final String MARRT = "marrt";
    private static final String SAFMARRT = "safmarrt";
    private static final String BMARRT = "bmarrt";

    private DirectedGraph<Waypoint, SpatialManeuver> graph;
    private Collection<SpaceRegion> obstacles;
    private Box4dRegion bounds;
    private SpatialPoint[] targets;
    private SpatialPoint[] starts;
    private double separation;
    private Trajectory[] trajectories;
    private SimulatedAgentEnvironment simulation = null;

    private RRTStarPlanner<JointWaypointState, JointGraphPath> rrt;
    private SingleAgentRRTStarSolver[] singleAgentSolvers;

    private String[] args;

    private String alg;

    private String scenario;

    private int cols;

    private int rows;

    private int maxTime;

    private int nAgents;

    private double obstacledRatio;
    private double avgObstacleSize;

    private int seed;

    private String experimentId;



    class SolutionListener implements Listener{
        long runtimeStart;

        public SolutionListener(long runtimeStart) {
            super();
            this.runtimeStart = runtimeStart;
        }

        @Override
        public void notifyNewSolution(EvaluatedTrajectory[] trajectories, boolean provedOptimal) {

            long runtime = (System.currentTimeMillis() - runtimeStart);
            long expandedStates = Counters.statesExpanded;
            long iterations = Counters.iterations;
            String costStr;


            if (trajectories != null) {
               MultiAgentScenarioCreator.this.trajectories = Trajectories.toTrajectoryArray(trajectories);
               double cost = Trajectories.evaluateCost(trajectories);
               costStr = String.format("%.4f", cost);
               LOGGER.debug(String.format(">>> New solution found. Time: %d ms Cost: %.2f Exp. states: %d", runtime, cost, expandedStates));
               updateTrajectoriesInSimulation();
            } else {
                costStr = "NA";
            }

            System.out.println(experimentId+";"+alg+";"+scenario+";"+cols+";"+rows+";"+maxTime+";"+nAgents+";"+separation+";"+obstacledRatio+";"+avgObstacleSize+";"+seed+";"+runtime+";"+expandedStates+";" + iterations + ";"+costStr+";"+provedOptimal);
//            System.out.println("greedySearch: " + Counters.greedySearchNum +";"+ Counters.greedySearchNanos/1e6 +";"+ Counters.greedySearchNanos/Counters.greedySearchNum+";"+ Counters.greedySearchNum/iterations);
//            System.out.println("CollisionChecks: " + Counters.collisionCheckApproxNum +"/"+ Counters.collisionCheckNum +";"+ Counters.collisionCheckNanos/1e6 +";"+ Counters.collisionCheckNanos/Counters.collisionCheckApproxNum);
        }

    };

    @Override
    public void init(String[] args) {

        this.args = args;

        // Set-up the logger
        String overrideLogFileName = null;
        //if (args.length>1) overrideLogFileName  = args[1];

        Properties prop = new Properties();

        String propFile = null;
        if ((new File("log4j.properties")).isFile()) {
            propFile = "log4j.properties";
        } else {
            if ((new File("resources" + File.separator + "log4j" + File.separator + "log4j.properties")).isFile()) {
                propFile = "resources" + File.separator + "log4j" + File.separator + "log4j.properties";
            }

        }
        if (propFile != null) {
            try {
                prop.load(new FileInputStream(propFile));
            } catch (Exception ex){
                ex.printStackTrace();
            }
            if (overrideLogFileName != null) {
                prop.setProperty("log4j.appender.R.File", overrideLogFileName);
            }
            PropertyConfigurator.configure(prop);

        }

    }

    public void createFromArgs() {
        //for (String arg : args) { System.out.println(arg); }
        String experimentId = args[1];
        String algorithm = args[2];
        String scenario  = args[3];
        int width = Integer.parseInt(args[4]);
        int height = Integer.parseInt(args[5]);
        int nAgents = Integer.parseInt(args[6]);
        double separation = Double.parseDouble(args[7]);
        double obstacledRatio = Double.parseDouble(args[8]);
        double obstacleSize = Double.parseDouble(args[9]);
        int seed = Integer.parseInt(args[10]);
        boolean showVis = Boolean.parseBoolean(args[11]);
        int timeoutMs = Integer.parseInt(args[12]);
        create(experimentId, algorithm, scenario, width, height, (int)(width + height), nAgents, separation, obstacledRatio, obstacleSize, seed, showVis, timeoutMs);
    }

    public void create(String experimentId, String alg, String scenario,
            int cols, int rows, int maxTime,
            int nAgents, double separation,
            double obstacledRatio, double obstacleSize, int seed, boolean showVis,
            long timeoutMs) {

        this.experimentId = experimentId;
        this.alg = alg;
        this.scenario = scenario;
        this.cols = cols;
        this.rows = rows;
        this.maxTime = maxTime;
        this.nAgents = nAgents;
        this.obstacledRatio = obstacledRatio;
        this.seed = seed;
        this.separation = separation;

        ToyWorldMultiAgentProblemInstance problem = null;
        if (scenario.equals(SUPERCONFLICT)) {
            problem = new ToyWorldSuperconflictProblemInstance(cols, rows, maxTime, nAgents, separation/2, obstacledRatio, obstacleSize, seed);
        } else if (scenario.equals(RANDOM)) {
            problem = new ToyWorldRandomProblemInstance(cols, rows, maxTime, nAgents, separation/2, obstacledRatio, obstacleSize, seed);
        } else {
            throw new RuntimeException("Invalid scenario specified");
        }

        graph = problem.getGraph();
        obstacles = problem.getObstacles();
        avgObstacleSize = problem.getAverageObstacleSize();
        bounds = problem.getBounds4d();
        starts = problem.getStarts();
        targets = problem.getTargets();
        separation = problem.getAgentSizeRadius()*2;
        trajectories = new Trajectory[nAgents];

        if (showVis) {
            createSimulation();
            createVisualization();
        }

        if (alg.equals(JA)) {
            solveJA(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(OD)) {
            solveOD(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(ODID)) {
            solveODID(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(OA)) {
            solveOA(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(MARRT)) {
            solveMARRT(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(SAFMARRT)) {
            solveSAFMARRT(graph, maxTime,  starts, targets, separation, timeoutMs);
        } else if (alg.equals(BMARRT)) {
            solveBMARRT(graph, maxTime,  starts, targets, separation, timeoutMs);
        }
    }

    private void solveJA(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        Waypoint[] startWaypoints = toWaypoints(graph, starts);
        Waypoint[] goalWaypoints = toWaypoints(graph, targets);

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());
        JointActionAStarSolver solver = new JointActionAStarSolver(startWaypoints, goalWaypoints, graph, separation, maxTime, 1.0);
        SearchResult result = solver.solveTrajectories(timeoutMs);
        listener.notifyNewSolution(result.getTrajectories(), result.isFinished());
    }

    private void solveOD(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        ODState[] startWaypoints = toAgentStates(graph, starts);
        Waypoint[] goalWaypoints = toWaypoints(graph, targets);

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());

        ODSolver solver = new ODSolver(startWaypoints, goalWaypoints, graph, separation, maxTime, 1.0);
        SearchResult result = solver.solveTrajectories(timeoutMs);

        listener.notifyNewSolution(result.getTrajectories(), result.isFinished());

    }

    private void solveODID(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        ODState[] startWaypoints = toAgentStates(graph, starts);
        Waypoint[] goalWaypoints = toWaypoints(graph, targets);

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());

        IDSolver solver = new IDSolver(startWaypoints, goalWaypoints, graph, separation, maxTime, 1.0);
        SearchResult result = solver.solve(timeoutMs);

        listener.notifyNewSolution(result.getTrajectories(), result.isFinished());

    }

    private void solveOA(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        ODState[] startStates = toAgentStates(graph, starts);
        Waypoint[] goalWaypoints = toWaypoints(graph, targets);

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());

        OASolver solver = new OASolver(startStates, goalWaypoints, graph, separation, maxTime, 1.0);
        solver.registerListener(listener);

        SearchResult result = solver.solve(timeoutMs);
        listener.notifyNewSolution(result.getTrajectories(), result.isFinished());
    }

    private void solveMARRT(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        Waypoint[] startWaypoints = toWaypoints(graph, starts);
        Waypoint[] goalWaypoints  = toWaypoints(graph, targets);

        double maxEdgeLength = 1.2*this.bounds.getXSize()*nAgents;
        double gamma = 1.2*this.bounds.getXSize()*nAgents;
//        System.out.println("gamma: " + gamma);
        double tryGoal = 0.1;

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());

        MultiAgentRRTStarSolver solver = new MultiAgentRRTStarSolver(graph, bounds,
                startWaypoints, goalWaypoints, separation, maxTime, 1.0,
                gamma, maxEdgeLength, tryGoal);

        rrt = solver.getRRTStarPlanner();

        solver.registerListener(listener);
        EvaluatedTrajectory[] trajs = solver.solve(timeoutMs);

        listener.notifyNewSolution(trajs, false);
    }

    private void solveSAFMARRT(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {
        /*
        Waypoint[] startWaypoints = toWaypoints(graph, starts);
        Waypoint[] goalWaypoints  = toWaypoints(graph, targets);

        double maxEdgeLength = maxTime;
        double gamma = 0.9*maxTime*(startWaypoints.length);
        double tryGoal = 0.1;

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());
        SingleAgentFedMultiAgentRRTStarSolver solver = new SingleAgentFedMultiAgentRRTStarSolver(graph, bounds,
                startWaypoints, goalWaypoints, separation, maxTime, 1.0,
                gamma, maxEdgeLength, tryGoal);

        rrt = solver.getRRTStarPlanner();
        singleAgentSolvers = solver.getSingleAgentSolvers();

        solver.registerListener(listener);
        EvaluatedTrajectory[] trajs = solver.solve(timeoutMs);

        listener.notifyNewSolution(trajs, false);*/

    }

    private void solveBMARRT(
            DirectedGraph<Waypoint, SpatialManeuver> graph, double maxTime,
            SpatialPoint[] starts, SpatialPoint[] targets, double separation, long timeoutMs) {

        Waypoint[] startWaypoints = toWaypoints(graph, starts);
        Waypoint[] goalWaypoints  = toWaypoints(graph, targets);

        double maxEdgeLength = 1.2*this.bounds.getXSize()*nAgents;
        double gamma = 1.2*this.bounds.getXSize()*nAgents;
        double tryGoal = 0.1;

        SolutionListener listener = new SolutionListener(System.currentTimeMillis());
        BiasedMultiAgentRRTStarSolver solver = new BiasedMultiAgentRRTStarSolver(graph, bounds,
                startWaypoints, goalWaypoints, separation, maxTime, 1.0,
                gamma, maxEdgeLength, tryGoal);

        rrt = solver.getRRTStarPlanner();
        singleAgentSolvers = solver.getSingleAgentSolvers();

        solver.registerListener(listener);
        EvaluatedTrajectory[] trajs = solver.solve(timeoutMs);

        listener.notifyNewSolution(trajs, false);

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

    private void createSimulation() {
        simulation  = new SimulatedAgentEnvironment();
    }

    private void updateTrajectoriesInSimulation() {
        if (simulation != null) {
            simulation.clearTrajectories();
            for (int i=0; i < trajectories.length; i++) {
                simulation.updateTrajectory("a"+i, trajectories[i]);
            }
        }
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

        KeyToggleLayer graphToggle = KeyToggleLayer.create("g");
        graphToggle.addSubLayer(GraphLayer.create(new GraphProvider<Waypoint, SpatialManeuver>() {

            @Override
            public Graph<Waypoint, SpatialManeuver> getGraph() {
                return graph;
            }}, Color.GRAY, Color.GRAY, 1, 4));
        graphToggle.setEnabled(true);

        VisManager.registerLayer(graphToggle);


        VisManager.registerLayer(NodeIdLayer.create(graph, Color.GRAY, 1, "n"));



        // X-Y TOP View ////////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, point.y);
            }
        });



//
//        // X-Z SIDE View ///////////////////////////////////////////////////////////////
//
//        createView( new ProjectionTo2d<TimePoint>() {
//
//            @Override
//            public Point2d project(TimePoint point) {
//                return new Point2d(point.x, -point.z + bounds.getYSize() + bounds.getZSize() + 2);
//            }
//        });
//
//        // Y-Z SIDE View ///////////////////////////////////////////////////////////////
//
//        createView( new ProjectionTo2d<TimePoint>() {
//
//            @Override
//            public Point2d project(TimePoint point) {
//                return new Point2d(point.z + bounds.getXSize() + 2, point.y);
//            }
//        });
//
//        // Y-T View ///////////////////////////////////////////////////////////////
//
//        createView( new ProjectionTo2d<TimePoint>() {
//
//            @Override
//            public Point2d project(TimePoint point) {
//                return new Point2d(-point.w - 2, point.y);
//            }
//        });
//
//
//        // X-T View ///////////////////////////////////////////////////////////////
//
//        createView( new ProjectionTo2d<TimePoint>() {
//
//            @Override
//            public Point2d project(TimePoint point) {
//                return new Point2d(point.x, -point.w  - 2);
//            }
//        });
//
//        ///////////////////////////////////////////////////////////////////////////
//
//        // Overlay
//        VisManager.registerLayer(VisInfoLayer.create());
//
//        // Simulation Control
//        VisManager.registerLayer(SimulationControlLayer.create(simulation));
    }

    private void createView(final ProjectionTo2d<TimePoint> projection) {


        VisManager.registerLayer(MissionsLayer.create(new MissionsProvider(){

            @Override
            public SpatialPoint[] getStarts() {
                return starts;
            }

            @Override
            public SpatialPoint[] getTargets() {
                return targets;
            }},

            new ProjectionTo2d<SpatialPoint>(){

                @Override
                public Point2d project(SpatialPoint point) {
                    return projection.project(new TimePoint(point, 0.0));
                }
            },
            Color.RED)
        );

        VisManager.registerLayer(JointOnGraphRRTStarLayer.create(new JointOnGraphRRTStarLayer.RRTStarProvider<JointWaypointState, JointGraphPath>()  {

            @Override
            public RRTStarPlanner<JointWaypointState, JointGraphPath> getRRTStar() {
                return rrt;
            }
        }, projection, this.starts.length, 0.0, bounds.getTSize()*this.starts.length, 3));

        VisManager.registerLayer(SingleAgentSolversLayer.create(new SingleAgentSolversProvider(){

            @Override
            public SingleAgentRRTStarSolver[] getSolvers() {
                return singleAgentSolvers;
            }},projection, 0, bounds.getTSize(), 2));

        VisManager.registerLayer(TrajectoriesLayer.create(new TrajectoriesProvider() {

            @Override
            public Trajectory[] getTrajectories() {
                return trajectories;
            }
        }, projection, 0.1, bounds.getCorner2().w, 't'));

        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

            @Override
            public Collection<SpaceTimeRegion> getSpaceTimeRegions() {
                LinkedList<SpaceTimeRegion> regions = new LinkedList<SpaceTimeRegion>();
                regions.add(bounds);
                return regions;
            }
        },
        projection,
        Color.BLACK, 1));
//
        // Obstacles
        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

            @Override
            public Collection<SpaceRegion> getSpaceRegions() {
                List<SpaceRegion> regions = new LinkedList<SpaceRegion>();
                regions.addAll(obstacles);
                return regions;
            }


        },
        projection,
        Color.BLACK, 2));

        VisManager.registerLayer(SimulatedCylindricAgentLayer.create(simulation.getAgentStorage(), projection, new TimeProvider() {

            @Override
            public double getTime() {
                return simulation.getTime();
            }
        }, separation/2, 0.1));
    }
  }
