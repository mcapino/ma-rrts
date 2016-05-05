package cz.agents.alite.trajectorytools.experiments;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.imageio.ImageIO;

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
import cz.agents.alite.trajectorytools.demo.ButtonLayer;
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
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;

public class AlternativePlannersUserComparison implements Creator {

    private static final int IMG_OFFSET = 620;

	private static final int WORLD_SIZE = 10;

//	private static final SpatialPoint START_POINT = new SpatialPoint(0, 0, 0);
//	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE, WORLD_SIZE, 0);

	private static final SpatialPoint START_POINT = new SpatialPoint(0, WORLD_SIZE/2, 0);
	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE, WORLD_SIZE/2, 0);

    private static final int PATH_SOLUTION_LIMIT = 5;

    private ObstacleGraphView graph;
    private static final AStarPlanner<SpatialPoint, DefaultWeightedEdge> planner = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();
    {
        planner.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
        @Override
            public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
                return current.distance(goal) + 0.01*Line2D.ptLineDist(START_POINT.x, START_POINT.y, TARGET_POINT.x, TARGET_POINT.y, current.x, current.y);
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
//        alternativePlanners.add( 
//                new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner) 
//                );
        alternativePlanners.add( 
                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new ObstacleExtensions<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
                );
//        alternativePlanners.add( 
//                new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>( planner )
//                );
        alternativePlanners.add( 
                new AlternativePlannerSelector<SpatialPoint, DefaultWeightedEdge>( new VoronoiDelaunayPlanner<SpatialPoint, DefaultWeightedEdge>(planner), PATH_SOLUTION_LIMIT)
                );
    }

    private List<List<PlannedPath<SpatialPoint, DefaultWeightedEdge>>> paths = new ArrayList<List<PlannedPath<SpatialPoint, DefaultWeightedEdge>>>();
    {
        for (int i=0; i<alternativePlanners.size(); i++) {
        	paths.add( new ArrayList<PlannedPath<SpatialPoint,DefaultWeightedEdge>>() );
        }
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

    private List<VisLayer> layers = new ArrayList<VisLayer>();
    
	protected int index = 40;

	private Random random = new Random();

	private Graph<SpatialPoint, DefaultWeightedEdge> originalGraph;

    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        originalGraph = SpatialGridFactory.create8WayGridSpatial(WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, 1.0);

        graph = ObstacleGraphView.createFromGraph(originalGraph, new ChangeListener() {
            @Override
            public void graphChanged() {
                replan();
            }
        } );

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

        for (int i = 0; i<paths.size(); i++) {
            layers.add(GraphPathLayer.create(graph, paths.get(i), 2, 4));
		}

        VisManager.registerLayer(ButtonLayer.create("Next...", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				for (int obstNum = 2; obstNum <= 16; obstNum+=2) {
					for (int i = 0; i < 5; i++) {
						generateResultImage(obstNum);
					}
				}
			}
		}, 780, 150, 170, 40));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {
    	for (int currentPlanner = 0; currentPlanner<alternativePlanners.size(); currentPlanner++) {
    		try {
    			paths.get(currentPlanner).clear();

    			paths.get(currentPlanner).addAll(
    					alternativePlanners.get(currentPlanner).planPath(
    							graph,
    							SpatialGraphs.getNearestVertex(graph, START_POINT),
    							SpatialGraphs.getNearestVertex(graph, TARGET_POINT)
    							) );
    		} catch (Exception e) {
    			System.out.println("Error: " + e.getMessage());
    			e.printStackTrace();
    			paths.get(currentPlanner).clear();
    		}
    	}
    }

    private List<SpatialPoint> generateRandomObstacles(int number) {
        
        List<SpatialPoint> obstacles = new ArrayList<SpatialPoint>(number);
    
	    while ( obstacles.size() < number ) {
	        int tries = 0;
            tryGenerate:
	        while (true) {
	            SpatialPoint randomPoint = new SpatialPoint(random .nextInt( WORLD_SIZE - 1 ) + 1, random.nextInt( WORLD_SIZE - 1 ) + 1, 0.0 );
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

	public void generateResultImage(int obstaclesNum) {
		final List<SpatialPoint> obstacles = generateRandomObstacles(obstaclesNum);
		for (SpatialPoint spatialPoint : obstacles) {
			graph.addObstacle(spatialPoint);
		}

		replan();
		
		int width = (alternativePlanners.size() ) * IMG_OFFSET;
		int height = 620;

		BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_BGR) ;
		Graphics2D graphics = (Graphics2D) image.getGraphics();

		graphics.translate(-50, -50);

		for (int i=0; i<alternativePlanners.size(); i++) {


			for (VisLayer visLayer : VisManager.getLayers()) {
				if (!(visLayer instanceof ButtonLayer)) {
		    		visLayer.paint(graphics);
				}
			}
			layers.get(i).paint(graphics);

			graphics.translate(IMG_OFFSET, 0);
		}

		graphics.translate(-alternativePlanners.size() * IMG_OFFSET, 0);

		try {
			ImageIO.write(image, "png", new File("test/test-"+obstaclesNum+"_"+(index ++)+".png"));
		} catch (IOException ex) {
			ex.printStackTrace();
		}

		for (SpatialPoint spatialPoint : obstacles) {
			graph.removeObstacle(spatialPoint);
		}
	}
}
