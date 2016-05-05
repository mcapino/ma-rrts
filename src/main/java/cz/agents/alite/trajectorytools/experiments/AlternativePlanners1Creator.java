package cz.agents.alite.trajectorytools.experiments;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.alterantiveplanners.AlternativePathPlanner;
import cz.agents.alite.trajectorytools.alterantiveplanners.AlternativePlannerSelector;
import cz.agents.alite.trajectorytools.alterantiveplanners.DifferentStateMetricPlanner;
import cz.agents.alite.trajectorytools.alterantiveplanners.ObstacleExtensions;
import cz.agents.alite.trajectorytools.alterantiveplanners.TrajectoryDistanceMaxMinMetricPlanner;
import cz.agents.alite.trajectorytools.alterantiveplanners.TrajectoryDistanceMetricPlanner;
import cz.agents.alite.trajectorytools.alterantiveplanners.VoronoiDelaunayPlanner;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView.ChangeListener;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.trajectorymetrics.DifferentStateMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.ObstacleAvoidanceMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectoryDistanceMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectoryMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectorySetMetrics;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;


public class AlternativePlanners1Creator implements Creator {

    private static final int NUM_OF_OBSTACLES_MIN = 2;
    private static final int NUM_OF_OBSTACLES_MAX = 16;
    private static final int NUM_OF_OBSTACLES_STEP = 2;
    private static final int NUM_OF_REPEATS = 5;
    
    private static final int NUM_OF_THREADS = 5;
    private static final ExecutorService executor = Executors.newFixedThreadPool(NUM_OF_THREADS);

    private static final int PATH_SOLUTION_LIMIT = 5;

    private static final int [] PATH_SOLUTION_LIMITS = new int [] {3, 5, 10};

    private static final int WORLD_SIZE = 10;

    private static final AStarPlanner<SpatialPoint, DefaultWeightedEdge> planner = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();
    {
        planner.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
        @Override
            public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
                return current.distance(goal) + ( current.x > current.y ? 0.1 : -0.1 );
            }
        });
    }
    
    private static final List<AlternativePathPlanner<SpatialPoint, DefaultWeightedEdge>> alternativePlanners = new ArrayList<AlternativePathPlanner<SpatialPoint,DefaultWeightedEdge>>();
    {
//        alternativePlanners.add( 
//                new DifferentStateMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT )
//                );
//        alternativePlanners.add( 
//                new TrajectoryDistanceMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT, 2)
//                );
//        alternativePlanners.add( 
//                new TrajectoryDistanceMaxMinMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT, 2 )
//                );
        alternativePlanners.add( 
                new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner) 
                );
//        alternativePlanners.add( 
//                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
//                );
//        alternativePlanners.add( 
//                new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>( planner )
//                );
//        alternativePlanners.add( 
//                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
//                );
    }

    private static final List<TrajectoryMetric<SpatialPoint, DefaultWeightedEdge>> trajectoryMetrics = new ArrayList<TrajectoryMetric<SpatialPoint,DefaultWeightedEdge>>();
    {
        trajectoryMetrics.add( 
                new DifferentStateMetric<SpatialPoint, DefaultWeightedEdge>()
                );
        trajectoryMetrics.add( 
                new TrajectoryDistanceMetric<SpatialPoint, DefaultWeightedEdge>()
                );
        trajectoryMetrics.add( 
                new ObstacleAvoidanceMetric<SpatialPoint, DefaultWeightedEdge>()
                );
    }

    Random random = new Random(123);
    
    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        
        try {
            // detailed results
            final BufferedWriter out = new BufferedWriter(new FileWriter( "results.csv"));
    
            // aggregated results
//            BufferedWriter aggrOut = new BufferedWriter(new FileWriter(
//                    "results_aggr.csv"));
    
            out.write("WORLD_SIZE;" + WORLD_SIZE + ";Repeats;" + NUM_OF_REPEATS + ";Obst. cases;" + ( (NUM_OF_OBSTACLES_MAX - NUM_OF_OBSTACLES_MIN)/NUM_OF_OBSTACLES_STEP + 1)  );
            out.newLine();
            out.write( "numObstacles;experiment name;planner;duration;num of paths;average path lenth" );
            for (TrajectoryMetric<SpatialPoint, DefaultWeightedEdge> metric : trajectoryMetrics) {
                out.write(";" + metric.getName());
            }
            if (PATH_SOLUTION_LIMITS != null) {
                for (int limit : PATH_SOLUTION_LIMITS) {
                    for (TrajectoryMetric<SpatialPoint, DefaultWeightedEdge> metric : trajectoryMetrics) {
                        out.write(';');
                        out.write( metric.getName() + " (" + limit + ")" );
                    }
                }
            }
            out.newLine();
    
            for (int numObstacles = NUM_OF_OBSTACLES_MIN; numObstacles <= NUM_OF_OBSTACLES_MAX; numObstacles+=NUM_OF_OBSTACLES_STEP) {
            
//                System.out.println("numObstacles: " + numObstacles);
                
                for (int repeat = 0; repeat < NUM_OF_REPEATS; repeat++) {
//                    System.out.println("repeat: " + repeat);
                       
                    final List<SpatialPoint> obstacles = generateRandomObstacles(numObstacles);

                    int plannerNum = 0;
                    for (final AlternativePathPlanner<SpatialPoint, DefaultWeightedEdge> planner : alternativePlanners) {
//                        System.out.println("planner.getName(): " + planner.getName());
                        final String runStr = numObstacles + "/" + repeat;

                        plannerNum++;
                        
                        final int curPlannerNum = plannerNum;
                        executor.execute(new Runnable() {
                            @Override
                            public void run() {
                                System.out.println("run: " + runStr);
                                runExperiment(out, obstacles, planner, curPlannerNum);
                            }
                        });
                    }
                    
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        executor.shutdown();
        try {
            executor.awaitTermination(Long.MAX_VALUE, TimeUnit.DAYS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println("Done!");
    }

    private List<SpatialPoint> generateRandomObstacles(int number) {
        
        List<SpatialPoint> obstacles = new ArrayList<SpatialPoint>(number);
    
	    while ( obstacles.size() < number ) {
	        int tries = 0;
            tryGenerate:
	        while (true) {
	            SpatialPoint randomPoint = new SpatialPoint(random.nextInt( WORLD_SIZE - 1 ) + 1, random.nextInt( WORLD_SIZE - 1 ) + 1, 0.0 );
    	        for (SpatialPoint point : obstacles) {
                    if (randomPoint.distance(point) < 1.5) {
                        tries ++;
                        if (tries > 100 ) {
                            tries = 0;
                            obstacles.clear();
                            System.out.println("reseting obstacles (experiment.generateRandomObstacles)");
                        }
                        continue tryGenerate;
                    }
                }
                obstacles.add(randomPoint);
                break;
	        }
	    }
        return obstacles;
    }
    
    private void runExperiment(
            final BufferedWriter out,
            final List<SpatialPoint> obstacles,
            final AlternativePathPlanner<SpatialPoint, DefaultWeightedEdge> planner, 
            int curPlannerNum) {
        Graph<Waypoint, SpatialManeuver> originalGraph = createGraph(); 

        ObstacleGraphView graph = ObstacleGraphView.createFromGraph(originalGraph, new ChangeListener() {
            @Override
            public void graphChanged() {
            }
        } );

        for (SpatialPoint obstacle : obstacles) {
            graph.addObstacle(obstacle);
        }

        long startTime = System.currentTimeMillis();
        Collection<PlannedPath<SpatialPoint, DefaultWeightedEdge>> paths = 
                planner.planPath(
                        graph, 
                        SpatialGraphs.getNearestVertex(graph, new SpatialPoint(0, 0, 0)),
                        SpatialGraphs.getNearestVertex(graph, new SpatialPoint(WORLD_SIZE, WORLD_SIZE, 0))
                        );
        long duration = System.currentTimeMillis() - startTime;

        double averageLength = 0;
        for (PlannedPath<SpatialPoint, DefaultWeightedEdge> path : paths) {
            if (path == null) {
                System.out.println("paths: " + paths);
                System.out.println("graph.getObstacles(): "
                        + graph.getObstacles());
                System.out.println( obstacles.size() + ";" + planner.getName() + "-" + obstacles.size() + ";" + planner.getName() + ";" + duration + ";" + paths.size() + ";" + averageLength);
            }
            averageLength += path.getWeight();
        }
        if (paths.size() != 0) {
            averageLength /= paths.size();
        }
        
        StringBuffer sb = new StringBuffer("" + obstacles.size() + ";" + curPlannerNum + "-" + planner.getName() + "-" + String.format("%03d", obstacles.size()) + ";" + planner.getName() + ";" + duration + ";" + paths.size() + ";" + averageLength);

        for (TrajectoryMetric<SpatialPoint, DefaultWeightedEdge> metric : trajectoryMetrics) {
            sb.append(';');
            sb.append( TrajectorySetMetrics.getPlanSetAvgDiversity(paths, metric) );
        }
        if (PATH_SOLUTION_LIMITS != null) {
            for (int limit : PATH_SOLUTION_LIMITS) {
                Collection<PlannedPath<SpatialPoint, DefaultWeightedEdge>> bestPaths;
                bestPaths = AlternativePlannerSelector.getShortestPaths(paths, limit);
                for (TrajectoryMetric<SpatialPoint, DefaultWeightedEdge> metric : trajectoryMetrics) {
                    sb.append(';');
                    sb.append( 
                            TrajectorySetMetrics.getPlanSetAvgDiversity(
                                    bestPaths, 
                                    metric ));
                }
            }
        }

        try {
            synchronized (out) {
                out.write( sb.toString() );

                out.newLine();
            }
            out.flush();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private Graph<Waypoint, SpatialManeuver> createGraph() {
        return SpatialGridFactory.create4WayGrid(WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, 1.0);
    }
}
