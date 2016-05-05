package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

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
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.trajectorymetrics.DifferentStateMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.ObstacleAvoidanceMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectoryDistanceMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectoryMetric;
import cz.agents.alite.trajectorytools.trajectorymetrics.TrajectorySetMetrics;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;

public class DemoAlternative1Creator implements Creator {

    private static final int WORLD_SIZE = 10;

    private static final int PATH_SOLUTION_LIMIT = 5;

	private static final SpatialPoint START_POINT = new SpatialPoint(0, 0, 0);
	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE, WORLD_SIZE, 0);

//	private static final SpatialPoint START_POINT = new SpatialPoint(0, WORLD_SIZE/2, 0);
//	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE, WORLD_SIZE/2, 0);

    private ObstacleGraphView graph;
    private List<PlannedPath<SpatialPoint, DefaultWeightedEdge>> paths = new ArrayList<PlannedPath<SpatialPoint,DefaultWeightedEdge>>();

    private static final AStarPlanner<SpatialPoint, DefaultWeightedEdge> planner = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();
    {
        planner.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
        @Override
            public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
            return current.distance(goal) + 0.01*Line2D.ptLineDist(START_POINT.x, START_POINT.y, TARGET_POINT.x, TARGET_POINT.y, current.x, current.y);
//                return current.distance(goal) + ( current.x > current.y ? 0.1 : -0.1 );
            }
        });
    }

    private static final List<AlternativePathPlanner<SpatialPoint, DefaultWeightedEdge>> alternativePlanners = new ArrayList<AlternativePathPlanner<SpatialPoint,DefaultWeightedEdge>>();
    {
        alternativePlanners.add( 
                new DifferentStateMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT )
                );
        alternativePlanners.add( 
                new TrajectoryDistanceMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT, 2)
                );
        alternativePlanners.add( 
                new TrajectoryDistanceMaxMinMetricPlanner<SpatialPoint, DefaultWeightedEdge>( planner, PATH_SOLUTION_LIMIT, 2 )
                );
        alternativePlanners.add( 
                new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner) 
                );
        alternativePlanners.add( 
                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
                );
        alternativePlanners.add( 
                new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>( planner )
                );
        alternativePlanners.add( 
                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
                );
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

    private int currentPlanner = 0;

	private Graph<SpatialPoint, DefaultWeightedEdge> originalGraph;
    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        originalGraph = SpatialGridFactory.create8WayGridSpatial(WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, 1.0);

        graph = new ObstacleGraphView(originalGraph, new ChangeListener() {
            @Override
            public void graphChanged() {
                replan();
            }
        } );

        System.out.println("originalGraph: " + originalGraph);
        System.out.println("graph: " + graph);

        graph.refresh();
        System.out.println("graph: " + graph);
        
        System.out.println("=========");
        createVisualization();
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768, 20, 20);
        VisManager.setSceneParam(new SceneParams() {
        	@Override
        	public Rectangle getWorldBounds() {
        		return new Rectangle(-492, -495, 1000, 1000);
        	}
        	@Override
        	public double getDefaultZoomFactor() {
        		return 50;
        	}
        });
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // graph with obstacles
        graph.createVisualization();

        // draw the shortest path
        VisManager.registerLayer(GraphPathLayer.create(graph, paths, 2, 4));

        String[] plannerNames = new String[alternativePlanners.size()];
        for (int i=0; i<alternativePlanners.size(); i++) {
        	plannerNames[i] = alternativePlanners.get(i).getName();
        }
        
        VisManager.registerLayer(ListLayer.create(plannerNames, new ListLayer.SelectionListener() {
			@Override
			public void selectedIndex(int index) {
				currentPlanner = index;
				replan();				
			}
		}, 700, 100, 350, 150));
        
        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {
    	try {
    		paths.clear();

    		long startTime = System.currentTimeMillis();

    		System.out.println("graph: " + graph);
    		
    		paths.addAll(
    				alternativePlanners.get(currentPlanner).planPath(
    						graph,
    						SpatialGraphs.getNearestVertex(graph, START_POINT),
    						SpatialGraphs.getNearestVertex(graph, TARGET_POINT)
    						) );

    		System.out.println("Time: " + (System.currentTimeMillis() - startTime) + " ms");

    		System.out.println("paths: " + paths.size());
    		for (PlannedPath<SpatialPoint, DefaultWeightedEdge> path : paths) {
    			System.out.println("path.getWeight(): " + path.getWeight());
    		}

    		for (TrajectoryMetric<SpatialPoint, DefaultWeightedEdge> metric : trajectoryMetrics) {
    			System.out.println(metric.getName() + ": " + TrajectorySetMetrics.getPlanSetAvgDiversity(paths, metric));
    		}

    	} catch (Exception e) {
    		System.out.println("Error: " + e.getMessage());
    		e.printStackTrace();
    		paths.clear();
    	}
    	
    	graph.refresh();
    }
}
