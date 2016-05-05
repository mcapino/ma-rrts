package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.alg.AllPathsIterator;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.trajectorytools.graph.PlanarGraph;
import cz.agents.alite.trajectorytools.graph.VoronoiDelaunayGraph;
import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.planner.PathPlanner;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.vis.GraphHolder;

public class VoronoiDelaunayPlanner<V extends SpatialPoint, E> implements AlternativePathPlanner<V, E> {
    private static int MAX_WORLD_SIZE = 10000;

    private final PathPlanner<V, E> planner;

    public VoronoiDelaunayPlanner( PathPlanner<V, E> planner ) {
        this.planner = planner;
    }

    @Override
    public Collection<PlannedPath<V, E>> planPath(
            GraphWithObstacles<V,E> graph, V startVertex,
            V endVertex) {

        VoronoiDelaunayGraph voronoiGraphAlg = new VoronoiDelaunayGraph();

		List<SpatialPoint> border = makeBorder(
				new SpatialPoint[] {
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( -MAX_WORLD_SIZE,  -MAX_WORLD_SIZE, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( MAX_WORLD_SIZE,  -MAX_WORLD_SIZE, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( MAX_WORLD_SIZE,  MAX_WORLD_SIZE, 0)),
		                SpatialGraphs.getNearestVertex(graph, new SpatialPoint( -MAX_WORLD_SIZE,  MAX_WORLD_SIZE, 0))
		        }, 
				new SpatialPoint[] {
		                SpatialGraphs.getNearestVertex(graph, startVertex),
		                SpatialGraphs.getNearestVertex(graph, endVertex)
				});

        PlanarGraph planarGraph = PlanarGraph.createPlanarGraphView((Graph<SpatialPoint, DefaultWeightedEdge>) graph);

        GraphHolder<SpatialPoint, DefaultWeightedEdge> voronoiGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();
        GraphHolder<SpatialPoint, DefaultWeightedEdge> delaunayGraph = new GraphHolder<SpatialPoint, DefaultWeightedEdge>();

        for (SpatialPoint obstacle : graph.getObstacles()) {
            voronoiGraphAlg.addObstacle(obstacle);
        }

        voronoiGraph.graph = voronoiGraphAlg.getVoronoiGraph(border);
        delaunayGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);

        List<PlannedPath<V, E>> paths = new ArrayList<PlannedPath<V,E>>();

        AllPathsIterator<SpatialPoint, DefaultWeightedEdge> pathsIt = new AllPathsIterator<SpatialPoint, DefaultWeightedEdge>(voronoiGraph.graph,
                startVertex,
                endVertex
                );

        while (pathsIt.hasNext()) {
            PlannedPath<SpatialPoint, DefaultWeightedEdge> voronoiPath = pathsIt.next();

            voronoiGraphAlg.removeDualEdges(delaunayGraph.graph, voronoiPath.getEdgeList());

//            System.out.println("delaunayGraph.graph.vertexSet(): " + delaunayGraph.graph.vertexSet());

//            PlanarGraph<E> planarGraphDelaunay = new PlanarGraph<E>(delaunayGraph.graph);
//
//            delaunayGraph.graph.removeVertex(startVertex);
//            delaunayGraph.graph.removeVertex(targetVertex);
//
//            for (E voronoiEdge: planPath.getEdgeList()) {
//                planarGraphDelaunay.removeCrossingEdges(voronoiEdge.getSource(), voronoiEdge.getTarget());
//            }
//
//            delaunayGraph.graph = planarGraphDelaunay;

            for (DefaultWeightedEdge edge : delaunayGraph.graph.edgeSet()) {
                planarGraph.removeCrossingEdges(delaunayGraph.graph.getEdgeSource(edge), delaunayGraph.graph.getEdgeTarget(edge));
            }

            PlannedPath<V, E> plannedPath = planner.planPath((Graph<V, E>) planarGraph,
                    startVertex,
                    endVertex
                    );

            if ( plannedPath != null ) {
                if ( !paths.contains(plannedPath) ) {
                    paths.add(plannedPath);
                }
            }

            graph.refresh();
            delaunayGraph.graph = voronoiGraphAlg.getDelaunayGraph(border);

        }

        return paths;
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

    @Override
    public String getName() {
        return "Voronoi-Delaunay";
    }

}
