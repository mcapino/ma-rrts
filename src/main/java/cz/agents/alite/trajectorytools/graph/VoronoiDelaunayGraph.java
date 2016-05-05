package cz.agents.alite.trajectorytools.graph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleGraph;

import cz.agents.alite.trajectorytools.graph.delaunay.Pnt;
import cz.agents.alite.trajectorytools.graph.delaunay.Triangle;
import cz.agents.alite.trajectorytools.graph.delaunay.Triangulation;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class VoronoiDelaunayGraph {
  
    private static int INITIAL_SIZE = 10000;     // Size of initial triangle

    /**
     * there are two cases each of them is solved better with different value 
     * (this is grid case problem only - use true for non grid cases)
     */
    private static boolean USE_STRAIGHT_DELAUNAY_EDGES_ONLY = true;

    private Triangle initialTriangle = new Triangle(
            new Pnt(-INITIAL_SIZE, -INITIAL_SIZE),
            new Pnt( INITIAL_SIZE, -INITIAL_SIZE),
            new Pnt(           0,  INITIAL_SIZE));  // Initial triangle
    private Triangulation dt;                      // Delaunay triangulation

    private Set<SpatialPoint> obstacles;

    private Graph<SpatialPoint, DefaultWeightedEdge> voronoiGraph = null;
    private Map<DefaultWeightedEdge, VoronoiEdge> voronoiEdges = new HashMap<DefaultWeightedEdge, VoronoiEdge>();

    private List<SpatialPoint> voronoiBorder = new ArrayList<SpatialPoint>();

    private Map<Pnt, SpatialPoint> delaunayVertexes = new HashMap<Pnt, SpatialPoint>();

    private Map<DefaultWeightedEdge, List<DefaultWeightedEdge>> dualEdges = new HashMap<DefaultWeightedEdge, List<DefaultWeightedEdge>>();

    public VoronoiDelaunayGraph() {       
        dt = new Triangulation(initialTriangle);
        obstacles = new HashSet<SpatialPoint>();
    }
    

    public void addObstacle(SpatialPoint obstacle) {
        obstacles.add(obstacle);
        dt.delaunayPlace( new Pnt(obstacle.x, obstacle.y) );
    }

    public void setObstacles(Set<SpatialPoint> obstacles) {
        this.obstacles = obstacles;
        dt = new Triangulation(initialTriangle);

        for (SpatialPoint point : obstacles) {
            dt.delaunayPlace( new Pnt(point.x, point.y) );
        }
    }

    public Graph<SpatialPoint, DefaultWeightedEdge> getVoronoiGraph(List<SpatialPoint> border) {
        Graph<SpatialPoint, DefaultWeightedEdge> graph = new SimpleGraph<SpatialPoint, DefaultWeightedEdge>(DefaultWeightedEdge.class);

        Map<Triangle, SpatialPoint> vertexes = new HashMap<Triangle, SpatialPoint>(); 
        
        for (Triangle triangle : dt) {
            Pnt circumcenter = triangle.getCircumcenter();
            
            SpatialPoint vertex = circumcenter.toPoint();
            vertexes.put(triangle, vertex);
            graph.addVertex( vertex );
        }
        
        voronoiEdges.clear();
        
        for (Triangle triangle : dt) {
            for (Triangle tri: dt.neighbors(triangle)) {
                SpatialPoint sourceVertex = vertexes.get(triangle);
                SpatialPoint targetVertex = vertexes.get(tri);

                if (!sourceVertex.equals(targetVertex)) {
                	DefaultWeightedEdge edge = graph.addEdge(sourceVertex, targetVertex);
                    voronoiEdges.put(edge, new VoronoiEdge(triangle, tri));
                }
            }
        }       

        voronoiBorder.clear();
        clipVoronoiGraph(graph, border);
        
        // remove edges of Voronoi graph crossing any obstacle
        removeObstaclesFromGraph(graph);
        
        voronoiGraph = graph;
        return voronoiGraph;
    }

    private void removeObstaclesFromGraph(Graph<SpatialPoint, DefaultWeightedEdge> graph) {
        List<DefaultWeightedEdge> toRemove = new LinkedList<DefaultWeightedEdge>();
        for (SpatialPoint obstacle : obstacles) {
            for (DefaultWeightedEdge edge : graph.edgeSet()) {
                // is the edge crossing the obstacle?
                SpatialPoint point = getClosestIntersection(obstacle, graph.getEdgeSource(edge), graph.getEdgeTarget(edge));
                if (point.distance(obstacle) < 0.001) {
                    toRemove.add(edge);
                }
            }
        }
        graph.removeAllEdges(toRemove);
    }

    public Graph<SpatialPoint, DefaultWeightedEdge> getDelaunayGraph(List<SpatialPoint> border) {
        
        if (voronoiGraph == null) {
            getVoronoiGraph(border);
        }
        
        Graph<SpatialPoint, DefaultWeightedEdge> graph = new SimpleGraph<SpatialPoint, DefaultWeightedEdge>(DefaultWeightedEdge.class);

        delaunayVertexes.clear(); 
        for (Triangle triangle : dt) {
            for (Pnt pnt : triangle) {
                SpatialPoint vertex = pnt.toPoint();
                graph.addVertex(vertex);
                delaunayVertexes.put(pnt, vertex);
            }
        }

        PlanarGraph voronoiPlanarGraph = PlanarGraph.createPlanarGraphView(voronoiGraph);

        for (DefaultWeightedEdge edge : voronoiGraph.edgeSet()) {
            List<DefaultWeightedEdge> edges = addDelaunayEdges(graph, voronoiPlanarGraph, edge);
            dualEdges.put(edge, edges);
        }

        return graph;
    }

    public void removeDualEdges(Graph<SpatialPoint, DefaultWeightedEdge> graph, List<DefaultWeightedEdge> edgeList) {
        for (DefaultWeightedEdge maneuver : edgeList) {
            List<DefaultWeightedEdge> dualEdges = getDualEdge(maneuver);
            if (dualEdges != null) {
				for (DefaultWeightedEdge edge : dualEdges) {
	                if (!graph.removeEdge(edge)) {
	                	System.out.println("Edge should be in the graph!" + edge);
	                }
	            }
            }
        }
    }


	public List<DefaultWeightedEdge> getDualEdge(DefaultWeightedEdge edge) {
		return dualEdges.get(edge);
	}
    
    private List<DefaultWeightedEdge> addDelaunayEdges(Graph<SpatialPoint, DefaultWeightedEdge> graph, PlanarGraph voronoiPlanarGraph, DefaultWeightedEdge edge) {

        List<DefaultWeightedEdge> newEdges = new ArrayList<DefaultWeightedEdge>();
        
        TemporaryEdge delaunayEdge = getDualEdgeToVoronoi( edge );

        if (USE_STRAIGHT_DELAUNAY_EDGES_ONLY && delaunayEdge != null) {
            newEdges.add( graph.addEdge(delaunayEdge.source, delaunayEdge.target) );
        } else {
            // it's a border edge
            SpatialPoint center = new SpatialPoint(
                    (graph.getEdgeSource(edge).x + graph.getEdgeTarget(edge).x) / 2.0, 
                    (graph.getEdgeSource(edge).y + graph.getEdgeTarget(edge).y) / 2.0,
                    0
                    );
            graph.addVertex(center);

            double minDist = Double.MAX_VALUE;
            SpatialPoint minObstacle = null;
            for (SpatialPoint obstacle : obstacles) {
            	if (minDist > center.distance(obstacle)) {
            		minDist = center.distance(obstacle);
            		minObstacle = obstacle;
            	}
            }
            if (minObstacle != null) {
            	newEdges.add( graph.addEdge(center, SpatialGraphs.getNearestVertex(graph, minObstacle )) );
            }
        }

        return newEdges;
    }

    private class TemporaryEdge {
        SpatialPoint source;
        SpatialPoint target;
    }
    
    private TemporaryEdge getDualEdgeToVoronoi(DefaultWeightedEdge edge) {
        VoronoiEdge voronoiEdge = voronoiEdges.get(edge);
        if (voronoiEdge == null) {
            return null;
        } else {
        
            ArrayList<Pnt> tmpCol = new ArrayList<Pnt>(voronoiEdge.source);
            tmpCol.retainAll(voronoiEdge.target);
            
            if (tmpCol.size() != 2) {
                throw new Error(voronoiEdge.source + " x " + voronoiEdge.target + " = " + tmpCol);
            }
            
            TemporaryEdge tempEdge = new TemporaryEdge();
            tempEdge.source = delaunayVertexes.get(tmpCol.get(0));
            tempEdge.target = delaunayVertexes.get(tmpCol.get(1));         
    
            return tempEdge;
        }
    }


    private void clipVoronoiGraph(Graph<SpatialPoint, DefaultWeightedEdge> graph, List<SpatialPoint> border) {
        PlanarGraph planarGraph = PlanarGraph.createPlanarGraphView(graph);
        
        SpatialPoint last = null;
        for (SpatialPoint vertex : border) {
            if (last != null) {
                voronoiBorder.addAll(
                    planarGraph.addLine(last, vertex, voronoiEdges)
                    );
            }
            last = vertex;
        }
        voronoiBorder.addAll(
                planarGraph.addLine(last, border.get(0), voronoiEdges)
                );
            
        removeOutsideVertices(graph, border);
    }

    private static <E> void removeOutsideVertices(Graph<SpatialPoint, E> graph,
            List<SpatialPoint> border) {
        
        List<SpatialPoint> toRemove = getOutsideVertices(graph, border);
        graph.removeAllVertices(toRemove);
    }

    private static <E> List<SpatialPoint> getOutsideVertices(
            Graph<SpatialPoint, E> graph, List<SpatialPoint> border) {
        SpatialPoint center = new SpatialPoint();
        for (SpatialPoint point : border) {
            center.x += point.x;
            center.y += point.y;
        }
        
        center.x /= border.size();
        center.y /= border.size();
        

        List<SpatialPoint> toRemove = new ArrayList<SpatialPoint>();
        for (SpatialPoint vertex : graph.vertexSet()) {
            SpatialPoint intersection = getBorderIntersection(center, vertex, border);
            if (intersection != null) {
                if (!intersection.epsilonEquals(vertex, 0.01)) {
                    toRemove.add( vertex );
                }
            }
        }
        return toRemove;
    }
    
    private static SpatialPoint getBorderIntersection(SpatialPoint point1, SpatialPoint point2, List<SpatialPoint> border) {
        SpatialPoint last = null;
        for (SpatialPoint vertex : border) {
            if (last != null) {
                SpatialPoint intersection = getIntersection(point1, point2, last, vertex);
                if (intersection != null && !intersection.epsilonEquals(point1, 0.001) && !intersection.epsilonEquals(point2, 0.001)) {
                    return intersection;
                }
            }
            last = vertex;
        }
        return getIntersection(point1, point2, last, border.get(0));
    }

    private static SpatialPoint getClosestIntersection(SpatialPoint point, SpatialPoint point1, SpatialPoint point2) {
        double u = ((point.x - point1.x) * (point2.x - point1.x) + (point.y - point1.y) * (point2.y - point1.y)) / point1.distanceSquared(point2);
        if (u < 0) {
            return point1;
        } else if (u > 1) {
            return point2;
        } else {
            double x = point1.x + u * ( point2.x - point1.x );
            double y = point1.y + u * ( point2.y - point1.y );

            return new SpatialPoint(x, y, 0);
        }
    }

    /**
     * intersection in 2D
     * 
     * @param point1
     * @param point2
     * @param point3
     * @param point4
     * @return
     */
    private static SpatialPoint getIntersection(SpatialPoint point1, SpatialPoint point2, SpatialPoint point3, SpatialPoint point4){
        double a1, a2, b1, b2, c1, c2;
        double r1, r2 , r3, r4;
        double denom;

        // Compute a1, b1, c1, where line joining points 1 and 2
        // is "a1 x + b1 y + c1 = 0".
        a1 = point2.y - point1.y;
        b1 = point1.x - point2.x;
        c1 = (point2.x * point1.y) - (point1.x * point2.y);

        // Compute r3 and r4.
        r3 = ((a1 * point3.x) + (b1 * point3.y) + c1);
        r4 = ((a1 * point4.x) + (b1 * point4.y) + c1);

        // Check signs of r3 and r4. If both point 3 and point 4 lie on
        // same side of line 1, the line segments do not intersect.
        if ((r3 != 0) && (r4 != 0) && same_sign(r3, r4)){
          return null;
        }

        // Compute a2, b2, c2
        a2 = point4.y - point3.y;
        b2 = point3.x - point4.x;
        c2 = (point4.x * point3.y) - (point3.x * point4.y);

        // Compute r1 and r2
        r1 = (a2 * point1.x) + (b2 * point1.y) + c2;
        r2 = (a2 * point2.x) + (b2 * point2.y) + c2;

        // Check signs of r1 and r2. If both point 1 and point 2 lie
        // on same side of second line segment, the line segments do
        // not intersect.
        if ((r1 != 0) && (r2 != 0) && (same_sign(r1, r2))){
          return null;
        }

        //Line segments intersect: compute intersection point.
        denom = (a1 * b2) - (a2 * b1);

        if (denom == 0) {
          return null;
        }

        return new SpatialPoint(((b1 * c2) - (b2 * c1)) / denom, ((a2 * c1) - (a1 * c2)) / denom, 0.0);
    }

    private static boolean same_sign(double a, double b){
        return (( a * b) >= 0);
    }
    
    private static class VoronoiEdge {
        Triangle source;
        Triangle target;

        public VoronoiEdge(Triangle source, Triangle target) {
            this.source = source;
            this.target = target;
        }
    }
}
