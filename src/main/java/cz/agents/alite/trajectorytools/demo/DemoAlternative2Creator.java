package cz.agents.alite.trajectorytools.demo;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.alg.AllPathsIterator;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView;
import cz.agents.alite.trajectorytools.graph.ObstacleGraphView.ChangeListener;
import cz.agents.alite.trajectorytools.graph.VoronoiDelaunayGraph;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.GraphHolder;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphPathLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;

public class DemoAlternative2Creator implements Creator {

    private static final int WORLD_SIZE = 9;

//	private static final SpatialPoint START_POINT = new SpatialPoint(0, 0, 0);
//	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE, WORLD_SIZE, 0);
	private static final SpatialPoint START_POINT = new SpatialPoint(WORLD_SIZE/2, 0, 0);
	private static final SpatialPoint TARGET_POINT = new SpatialPoint(WORLD_SIZE/2, WORLD_SIZE, 0);

	private static final int NUM_OF_RANDOM_OBSTACLES = 0;

    // shows one trajectory with voronoi and delaunay graphs
    private static final boolean DEBUG_VIEW = true;
    private int currentPath = 0;

    private ObstacleGraphView graph;
    private List<PlannedPath<SpatialPoint, DefaultWeightedEdge>> paths = new ArrayList<PlannedPath<SpatialPoint, DefaultWeightedEdge>>();

    VoronoiDelaunayGraph voronoiGraphAlg = new VoronoiDelaunayGraph();
    GraphHolder<SpatialPoint, DefaultWeightedEdge> voronoiGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();
    GraphHolder<SpatialPoint, DefaultWeightedEdge> delaunayGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();
    GraphHolder<SpatialPoint, DefaultWeightedEdge> otherGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();

    private static final AStarPlanner<SpatialPoint, DefaultWeightedEdge> planner = new AStarPlanner<SpatialPoint, DefaultWeightedEdge>();
    {
        planner.setHeuristicFunction(new HeuristicFunction<SpatialPoint>() {
        @Override
            public double getHeuristicEstimate(SpatialPoint current, SpatialPoint goal) {
                return current.distance(goal) + ( current.x > current.y ? 0.1 : -0.1 );
            }
        });
    }

//    private static final ObstacleExtensions alternativePlanner = new ObstacleExtensions(planner);
    private List<SpatialPoint> border;

//    private static final AlternativePathPlanner<SpatialWaypoint, SpatialManeuver> alternativePlanner = new TrajectoryDistanceMetric<SpatialWaypoint, SpatialManeuver>( planner );
//    private static final AlternativePathPlanner<SpatialWaypoint, SpatialManeuver> alternativePlanner = new DifferentStateMetric<SpatialWaypoint, SpatialManeuver>( planner );


    @Override
    public void init(String[] args) {
    }

    @Override
    public void create() {
        Graph<Waypoint, SpatialManeuver> originalGraph = SpatialGridFactory.create4WayGrid(WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, WORLD_SIZE, 1.0);

        graph = ObstacleGraphView.createFromGraph(originalGraph, new ChangeListener() {
            @Override
            public void graphChanged() {
            	currentPath = 0;
                replan();
            }
        } );

		border = makeBorder(
				new SpatialPoint[] {
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( 0,  0, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( WORLD_SIZE,  0, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( WORLD_SIZE,  WORLD_SIZE, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( 0,  WORLD_SIZE, 0))
		        }, 
				new SpatialPoint[] {
		                SpatialGraphs.getNearestVertex(graph, START_POINT),
		                SpatialGraphs.getNearestVertex(graph, TARGET_POINT)
				});

		System.out.println("border: " + border);
		
        createVisualization();

        List<SpatialPoint> obstacles = generateRandomObstacles(NUM_OF_RANDOM_OBSTACLES);
        for (SpatialPoint obstacle : obstacles) {
            graph.addObstacle(SpatialGraphs.getNearestVertex(graph, obstacle));
        }
    }

    private List<SpatialPoint> generateRandomObstacles(int number) {
        List<SpatialPoint> obstacles = new ArrayList<SpatialPoint>(number);
        for (int i=0; i<number; i++) {
            obstacles.add(new SpatialPoint(Math.random() * WORLD_SIZE, Math.random() * WORLD_SIZE, 0.0 ));
        }

        return obstacles;
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

        if (DEBUG_VIEW) {
            VisManager.registerLayer(GraphLayer.create(voronoiGraph, Color.GREEN, Color.GREEN, 1, 4));
            VisManager.registerLayer(GraphLayer.create(otherGraph, Color.MAGENTA, Color.MAGENTA, 1, 4, 0.02));
            VisManager.registerLayer(GraphLayer.create(delaunayGraph, Color.BLUE, Color.BLUE, 1, 4, 0.04));
        }

        // draw the shortest path
        VisManager.registerLayer(GraphPathLayer.create(voronoiGraph, paths, 2, 4));

        VisManager.registerLayer(ButtonLayer.create("Previous trajectory", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
            	currentPath --;
                replan();
			}
		}, 780, 100, 170, 40));

        VisManager.registerLayer(ButtonLayer.create("Next trajectory", new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
            	currentPath ++;
                replan();
			}
		}, 780, 150, 170, 40));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    protected void replan() {
        voronoiGraphAlg.setObstacles(graph.getObstacles());

        delaunayGraph.graph = null;
        voronoiGraph.graph = voronoiGraphAlg.getVoronoiGraph(border);
        delaunayGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);

        otherGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);

        paths.clear();

        SpatialPoint startVertex = SpatialGraphs.getNearestVertex(graph, START_POINT);
        SpatialPoint targetVertex =  SpatialGraphs.getNearestVertex(graph, TARGET_POINT);

        AllPathsIterator<SpatialPoint, DefaultWeightedEdge> pathsIt
            = new AllPathsIterator<SpatialPoint, DefaultWeightedEdge>(voronoiGraph.graph,
                startVertex,
                targetVertex
                );

        while (pathsIt.hasNext()) {
            for (int i=0; i<currentPath; i++) {
                if (pathsIt.hasNext()) pathsIt.next();
            }

            if (!pathsIt.hasNext()) {
                break;
            }

            PlannedPath<SpatialPoint, DefaultWeightedEdge> planPath = pathsIt.next();
            if (DEBUG_VIEW) {
                paths.add(planPath);
            }

            delaunayGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);

            if (delaunayGraph.graph != null) {
	            voronoiGraphAlg.removeDualEdges(delaunayGraph.graph, planPath.getEdgeList());
	
	            graph.refresh();
	
	            for (DefaultWeightedEdge edge : delaunayGraph.graph.edgeSet()) {
	                graph.removeCrossingEdges(delaunayGraph.graph.getEdgeSource(edge), delaunayGraph.graph.getEdgeTarget(edge));
	            }
            }
            
            planPath = planner.planPath(graph,
                    startVertex,
                    targetVertex
                    );



            if ( planPath!= null ) {
//                if ( !paths.contains(planPath) ) {
                    paths.add(planPath);
//                } else {
//                    System.out.println("Path already found: " + planPath);
//                }
            }
            if (DEBUG_VIEW) {
                break;
            }
        }

        System.out.println("paths.size(): " + paths.size());
    }
    
	private List<SpatialPoint> makeBorder(SpatialPoint[] border, SpatialPoint[] points) {
		List<SpatialPoint> newBorder = new LinkedList<SpatialPoint>(Arrays.asList(border));

		for (SpatialPoint point : points) {
			if (!newBorder.contains(point)) {
				double minDist = Double.MAX_VALUE;
				int closestEdge = 0;
				
				for (int i=0; i<newBorder.size(); i++) {
					SpatialPoint point1 = newBorder.get(i);
					SpatialPoint point2 = newBorder.get((i+1) % newBorder.size());
					double dist = Line2D.ptSegDist(point1.x, point1.y, point2.x, point2.y, point.x, point.y);
					if (dist < minDist) {
						minDist = dist;
						closestEdge = i;
					}
				}
				newBorder.add(closestEdge+1, point);
			}
		}
		return newBorder;
	}
}
