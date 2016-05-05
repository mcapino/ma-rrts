package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.jgrapht.alg.AllPathsIterator;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.VoronoiDelaunayGraph;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.PolygonRegion;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.GraphHolder;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer;
import cz.agents.alite.trajectorytools.vis.PolygonRegionLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.element.FilledStyledCircle;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.FilledStyledCircleElements;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.FilledStyledCircleImpl;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.alite.vis.layer.common.VisualInteractionLayer;
import cz.agents.alite.vis.layer.common.VisualInteractionLayer.VisualInteractionProvidingEntity;
import cz.agents.alite.vis.layer.terminal.FilledStyledCircleLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;

public class DemoAlternative3Creator implements Creator {

    private static final int NUM_OF_RANDOM_OBSTACLES = 0;

    private static final int WORLD_SIZE = 500;

    // shows one trajectory with voronoi and delaunay graphs
    private static final boolean DEBUG_VIEW = true;
    private int currentPath = 0;

    private List<PlannedPath<SpatialPoint, DefaultWeightedEdge>> paths = new ArrayList<PlannedPath<SpatialPoint, DefaultWeightedEdge>>();

    VoronoiDelaunayGraph voronoiGraphAlg = new VoronoiDelaunayGraph();
    GraphHolder<SpatialPoint, DefaultWeightedEdge> voronoiGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();
    GraphHolder<SpatialPoint, DefaultWeightedEdge> delaunayGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();

    private static final AStarPlanner<SpatialPoint, DefaultWeightedEdge> planner = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();

	{
        planner.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
        @Override
            public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
                return current.distance(goal) + ( current.x > current.y ? 0.1 : -0.1 );
            }
        });
    }

	private double minX = -400;
	private double maxX = 400;
	private double minY = -400;
	private double maxY = 400;

	final SpatialPoint startPoint = new SpatialPoint(-300, -300, 0);
	final SpatialPoint targetPoint = new SpatialPoint(300, 300, 0);

    
//    private static final ObstacleExtensions alternativePlanner = new ObstacleExtensions(planner);
    private List<SpatialPoint> border;

	private Map<DefaultWeightedEdge, PolygonRegion> backupObstacleMap = new HashMap<DefaultWeightedEdge, PolygonRegion>();
	private Map<DefaultWeightedEdge, PolygonRegion> obstacleMap = new HashMap<DefaultWeightedEdge, PolygonRegion>();

	private List<SpatialPoint> obstacles = new ArrayList<SpatialPoint>();
	
//    private static final AlternativePathPlanner<SpatialWaypoint, SpatialManeuver> alternativePlanner = new TrajectoryDistanceMetric<SpatialWaypoint, SpatialManeuver>( planner );
//    private static final AlternativePathPlanner<SpatialWaypoint, SpatialManeuver> alternativePlanner = new DifferentStateMetric<SpatialWaypoint, SpatialManeuver>( planner );


    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        border = Arrays.asList(new SpatialPoint[] {
                new SpatialPoint( minX,  minY, 0),
                new SpatialPoint( maxX,  minY, 0),
                new SpatialPoint( maxX,  maxY, 0),
                new SpatialPoint( minX,  maxY, 0)
        });

        createVisualization();
        
        obstacles.addAll( generateRandomObstacles(NUM_OF_RANDOM_OBSTACLES) );
    }

    private List<SpatialPoint> generateRandomObstacles(int number) {
        List<SpatialPoint> obstacles = new ArrayList<SpatialPoint>(number);
        for (int i=0; i<number; i++) {
            obstacles.add(new SpatialPoint(Math.random() * WORLD_SIZE, Math.random() * WORLD_SIZE, 0.0 ));
        }

        return obstacles;
    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1000, 950, 20, 20);
        VisManager.setSceneParam(new SceneParams() {
        	@Override
        	public Rectangle getWorldBounds() {
        		return new Rectangle(-500, -500, 1000, 1000);
        	}
        	@Override
        	public double getDefaultZoomFactor() {
        		return .5;
        	}
        });
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // graph with obstacles
//        graph.createVisualization();

        if (DEBUG_VIEW) {
            VisManager.registerLayer(GraphLayer.create(voronoiGraph, Color.GREEN, Color.GREEN, 1, 4));
            VisManager.registerLayer(GraphLayer.create(delaunayGraph, Color.BLUE, Color.BLUE, 1, 4, 5));

            VisManager.registerLayer(GraphPathLayer.create(voronoiGraph, paths, 2, 4));

            // interaction
            VisManager.registerLayer(VisualInteractionLayer.create(new VisualInteractionProvidingEntity() {

                @Override
                public void interactVisually(double x, double y, MouseEvent e) {

                	if (y < -400) {
                    	return;
                    }

                    SpatialPoint point = new SpatialPoint(x, y, 0);
                    
                    if (e.getButton() == MouseEvent.BUTTON1) {
                        obstacles.add(point);
                    } else if (e.getButton() == MouseEvent.BUTTON3) {
                    	SpatialPoint closestObstacle = null;
                    	double closestDist = Double.MAX_VALUE;
                    	for (SpatialPoint obstacle : obstacles) {
							if (obstacle.distance(point) < closestDist) {
								 closestDist = obstacle.distance(point);
								 closestObstacle = obstacle;
							}
						}
                    	
                    	obstacles.remove(closestObstacle);
                    } else {
                        return;
                    }
                    currentPath = 0;
                    replan();
                }

                @Override
                public String getName() {
                    return "Obstacles layer : ClickableObstaclesLayer";
                }
            }));

            // drawing obstacles
            VisManager.registerLayer(FilledStyledCircleLayer.create(new FilledStyledCircleElements() {

            	protected final Color OBSTACLE_COLOR = Color.GREEN;
                
                @Override
                public int getStrokeWidth() {
                    return 1;
                }

                @Override
                public Color getColor() {
                    return Color.GREEN.darker();
                }

                @Override
                public Color getFillColor() {
                    return OBSTACLE_COLOR;
                }

                @Override
                public Iterable<? extends FilledStyledCircle> getCircles() {
                    List<FilledStyledCircle> circles = new ArrayList<FilledStyledCircle>(obstacles.size());
                    for (final SpatialPoint point : obstacles) {
                        FilledStyledCircle circle = new FilledStyledCircleImpl(point, 20, OBSTACLE_COLOR, OBSTACLE_COLOR.darker());
                        circles.add(circle);
                    }
                    return circles;
                }
            }));
        }

        VisManager.registerLayer(PolygonRegionLayer.create(obstacleMap, Color.ORANGE, 3));
        VisManager.registerLayer(PolygonRegionLayer.create(backupObstacleMap, Color.CYAN, 2));

        VisManager.registerLayer(ButtonLayer.create("Next trajectory", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
            	currentPath ++;
                replan();
			}
		}, 600, 20, 170, 40));

        VisManager.registerLayer(ButtonLayer.create("Previous trajectory", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
            	currentPath --;
                replan();
			}
		}, 800, 20, 170, 40));

        VisManager.registerLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                LinkedList<Line> lines = new LinkedList<Line>();
                lines.add(new LineImpl(startPoint, targetPoint));
                return lines;
            }

            @Override
            public int getStrokeWidth() {
                return 1;
            }

            @Override
            public Color getColor() {
                return Color.MAGENTA;
            }

        }));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {
		VoronoiDelaunayGraph voronoiGraphAlg = new VoronoiDelaunayGraph();
	
		for (SpatialPoint obstacle : obstacles) {
			voronoiGraphAlg.addObstacle( obstacle );
		}
		
        voronoiGraph.graph = voronoiGraphAlg.getVoronoiGraph(border);
        delaunayGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);
	
        obstacleMap.clear();
        for (DefaultWeightedEdge edge : delaunayGraph.graph.edgeSet()) {
        	SpatialPoint source = delaunayGraph.graph.getEdgeSource(edge);
        	SpatialPoint target = delaunayGraph.graph.getEdgeTarget(edge);
        	System.out.println("source: " + source);
        	System.out.println("target: " + target);
        	PolygonRegion obstacle =  
						new PolygonRegion( new SpatialPoint [] {
		        			new SpatialPoint(source.x, source.y, source.z - 100),
		        			new SpatialPoint(source.x, source.y, source.z + 100),
		        			new SpatialPoint(target.x, target.y, target.z + 100),
		        			new SpatialPoint(target.x, target.y, target.z - 100)
						});
			obstacleMap.put(edge, obstacle);
		}

		SpatialPoint nearestPoint = findNearestVisibleVoronoiPoint(startPoint, obstacleMap.values());
		voronoiGraph.graph.addVertex(startPoint);
		voronoiGraph.graph.addEdge(startPoint, nearestPoint);

		nearestPoint = findNearestVisibleVoronoiPoint(targetPoint, obstacleMap.values());
		voronoiGraph.graph.addVertex(targetPoint);
		voronoiGraph.graph.addEdge(targetPoint, nearestPoint);
        
        backupObstacleMap.clear();
        paths.clear();

		AllPathsIterator<SpatialPoint, DefaultWeightedEdge> pathsIt = new AllPathsIterator<SpatialPoint, DefaultWeightedEdge>(voronoiGraph.graph,
        		startPoint,
        		targetPoint
                );

        while (pathsIt.hasNext()) {

            for (int i=0; i<currentPath; i++) {
                if (pathsIt.hasNext()) pathsIt.next();
            }

            if (!pathsIt.hasNext()) {
                break;
            }

            PlannedPath<SpatialPoint, DefaultWeightedEdge> voronoiPath = pathsIt.next();

            if (DEBUG_VIEW) {
                paths.add(voronoiPath);
            }

            for (DefaultWeightedEdge edge : voronoiPath.getEdgeList()) {
            	//TODO voronoiGraphAlg.getDualEdge( edge ) -> dualObstacle
				List<DefaultWeightedEdge> dualEdges = voronoiGraphAlg.getDualEdge( edge );
				if (dualEdges != null) {
					for (DefaultWeightedEdge dualEdge : dualEdges) {
						PolygonRegion removed = obstacleMap.remove( dualEdge );
						if (removed == null) {
							System.out.println("!!!! dualEdge: " + dualEdge);
							System.out.println("dualEdges: " + dualEdges);
						}
						backupObstacleMap.put(dualEdge, removed);
					}
				} else {
					// ok for the first and last edge
					System.out.println("Does not have a dual edge: " + edge);
				}
			}
            
    		System.out.println("obstacleMap: " + obstacleMap);

    		if (DEBUG_VIEW) {
    			break;
    		}

    		obstacleMap.putAll(backupObstacleMap);
            backupObstacleMap.clear();
        }
    }

    public SpatialPoint findNearestVisibleVoronoiPoint(SpatialPoint point, Collection<PolygonRegion> obstacles) {
		SpatialPoint nearestPoint = null;
        double minDist = Double.MAX_VALUE;
        for (SpatialPoint curPoint : voronoiGraph.graph.vertexSet()) {
        	if (minDist > point.distance(curPoint) && pointsAreVisible(curPoint, point, obstacles)) {
        		minDist = point.distance(curPoint);
        		nearestPoint = curPoint;
        	}
        }
		return nearestPoint;
	}

	private boolean pointsAreVisible(SpatialPoint point1,
			SpatialPoint point2, Collection<PolygonRegion> obstacles) {
		for (PolygonRegion polygonRegion : obstacles) {
			if (polygonRegion.intersectsLine(new TimePoint(point1, 0), new TimePoint(point2, 0))) {
				return false;
			}
		}
		return true;
	}
}
