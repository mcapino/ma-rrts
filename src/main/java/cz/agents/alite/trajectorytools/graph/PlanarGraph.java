package cz.agents.alite.trajectorytools.graph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.GraphDelegator;
import org.jgrapht.graph.SimpleGraph;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class PlanarGraph extends GraphDelegator<SpatialPoint, DefaultWeightedEdge> {
    private static final long serialVersionUID = -7039093249594157867L;

    protected PlanarGraph(Graph<SpatialPoint, DefaultWeightedEdge> graph) {
        super(graph);
    }


    /**
     * Creates a planar view over the given graph.
     * 
     * @param graph
     */
    public static PlanarGraph createPlanarGraphView(Graph<SpatialPoint, DefaultWeightedEdge> graph) {
        return new PlanarGraph(graph);
    }

    /**
     * Creates new graph - it's not a view
     * 
     * @param graph
     * @return
     */
    public static <V extends SpatialPoint, E> PlanarGraph createPlanarGraphCopy(Graph<V, E> graph) {
        Graph<SpatialPoint, DefaultWeightedEdge> planarGraph = new SimpleGraph<SpatialPoint, DefaultWeightedEdge>(DefaultWeightedEdge.class);
        Map<V, SpatialPoint> vertexes = new HashMap<V, SpatialPoint>();
        for (V vertex : graph.vertexSet()) {
            SpatialPoint point = new SpatialPoint(vertex.x, vertex.y, vertex.z);
            planarGraph.addVertex(point);
            vertexes.put(vertex, point);
        }
        
        for (E edge : graph.edgeSet()) {
            if ( !graph.getEdgeSource(edge).equals( graph.getEdgeTarget(edge) ) ) {
                planarGraph.addEdge(
                        vertexes.get( graph.getEdgeSource(edge) ), 
                        vertexes.get( graph.getEdgeTarget(edge) ));
            }
        }

        return new PlanarGraph(planarGraph);
    }

    @Override
    public DefaultWeightedEdge addEdge(SpatialPoint sourceVertex, SpatialPoint targetVertex) {
        addLine(sourceVertex, targetVertex, null);
        return null;
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public List<SpatialPoint> addLine(SpatialPoint sourceVertex, SpatialPoint targetVertex, Map edgeMap) {
        List<SpatialPoint> line = new LinkedList<SpatialPoint>(Arrays.asList(sourceVertex, targetVertex));

        //
        // Find intersections of edges with the line,
        // add there new points and split the edges.
        //

        List<DefaultWeightedEdge> toRemove = new ArrayList<DefaultWeightedEdge>();
        Map<DefaultWeightedEdge, SpatialPoint> toAdd = new HashMap<DefaultWeightedEdge, SpatialPoint>();
        for (DefaultWeightedEdge edge : edgeSet()) {

            SpatialPoint edgeSource = getEdgeSource(edge);
            SpatialPoint edgeTarget = getEdgeTarget(edge);
            SpatialPoint intersection = addLineIntersection(edgeSource, edgeTarget, line);

            if (intersection != null) {
                if (intersection.equals(edgeSource) || intersection.equals(edgeTarget)) {
                    // ok
                } else {
                    toRemove.add(edge);
                    addVertex(intersection);
                    toAdd.put( edge, intersection );
                }
            }
        }

        removeAllEdges(toRemove);

        for (Entry<DefaultWeightedEdge, SpatialPoint> entry : toAdd.entrySet()) {
            DefaultWeightedEdge edge1 = super.addEdge(getEdgeSource(entry.getKey()), entry.getValue());
            DefaultWeightedEdge edge2 = super.addEdge(entry.getValue(), getEdgeTarget(entry.getKey()));

            if (edgeMap != null) {
                Object edgeInfo = edgeMap.get(entry.getKey());
                if (edgeInfo != null) {
                    edgeMap.put(edge1, edgeInfo);
                    edgeMap.put(edge2, edgeInfo);
                }
            }
        }

        SpatialPoint last = null;
        for (SpatialPoint vertex : line) {
            if (!containsVertex(vertex)) {
                addVertex(vertex);
            }
            if (last != null) {
                super.addEdge(last, vertex);
                super.addEdge(vertex, last);
            }
            last = vertex;
        }

        return line;
    }

    public int countCrossingEdges(SpatialPoint point1, SpatialPoint point2) {
        int counter = 0;
        for (DefaultWeightedEdge edge : edgeSet()) {

            SpatialPoint edgeSource = getEdgeSource(edge);
            SpatialPoint edgeTarget = getEdgeTarget(edge);
            SpatialPoint intersection = getIntersection(point1, point2, edgeSource, edgeTarget);

            if (intersection != null) {
                counter++;
            }
        }
        return counter;
    }


    public void removeCrossingEdges(SpatialPoint point1, SpatialPoint point2) {
        List<DefaultWeightedEdge> toRemove = new ArrayList<DefaultWeightedEdge>();
        for (DefaultWeightedEdge edge : edgeSet()) {

            SpatialPoint edgeSource = getEdgeSource(edge);
            SpatialPoint edgeTarget = getEdgeTarget(edge);
            SpatialPoint intersection = getIntersection(point1, point2, edgeSource, edgeTarget);

            if (intersection != null) {
                toRemove.add(edge);
            }
        }

        removeAllEdges(toRemove);
    }

    static SpatialPoint addLineIntersection(SpatialPoint point1, SpatialPoint point2, List<SpatialPoint> border) {
        SpatialPoint last = null;
        int index = 0;
        for (SpatialPoint vertex : border) {
            if (last != null) {
                SpatialPoint intersection = getIntersection(point1, point2, last, vertex);
                if (intersection != null) {
                    if (intersection.epsilonEquals(last, 0.001)) {
                        return last;
                    } else if (intersection.epsilonEquals(vertex, 0.001)) {
                        return vertex;
                    } else if (intersection.epsilonEquals(point1, 0.001)) {
                        border.add(index, point1);
                        return point1;
                    } else if (intersection.epsilonEquals(point2, 0.001)) {
                        border.add(index, point2);
                        return point2;
                    } else {
                        border.add(index, intersection);
                        return intersection;
                    }
                }
            }
            last = vertex;
            index++;
        }
        return null;
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
    static SpatialPoint getIntersection(SpatialPoint point1, SpatialPoint point2, SpatialPoint point3, SpatialPoint point4) {
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

    static boolean same_sign(double a, double b){
        return (( a * b) >= 0);
    }
}