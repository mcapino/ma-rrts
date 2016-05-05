package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.LinkedList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.Point;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.aggregation.PointElements;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.element.implemetation.PointImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;
import cz.agents.alite.vis.layer.terminal.PointLayer;

public class GraphLayer extends AbstractLayer {

    public static interface GraphProvider<V, E> {
        Graph<V, E> getGraph();
    }

    GraphLayer() {
    }


    public static <V extends SpatialPoint,E> VisLayer create(final GraphProvider<V, E> graphProvider, final Color edgeColor, final Color vertexColor,
            final int edgeStrokeWidth, final int vertexStrokeWidth, final double offset) {

        return create(graphProvider, new ProjectionTo2d<V>() {

            @Override
            public Point2d project(V point) {
                return new Point2d(point.x + offset, point.y + offset);
            }

        }, edgeColor, vertexColor, edgeStrokeWidth, vertexStrokeWidth);

    }

    public static <V extends SpatialPoint,E> VisLayer create(final GraphProvider<V, E> graphProvider, final Color edgeColor, final Color vertexColor,
            final int edgeStrokeWidth, final int vertexStrokeWidth) {
        return create(graphProvider, edgeColor, vertexColor, edgeStrokeWidth, vertexStrokeWidth, 0.0);
    }

    public static <V, E> VisLayer create(final GraphProvider<V, E> graphProvider, final ProjectionTo2d<V> projection,  final Color edgeColor, final Color vertexColor,
            final int edgeStrokeWidth, final int vertexStrokeWidth) {
        GroupLayer group = GroupLayer.create();

        // edges
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                Graph<V, E> graph = graphProvider.getGraph();
                LinkedList<Line> lines = new LinkedList<Line>();
                for (E edge : graph.edgeSet()) {
                    Point2d source = projection.project( graph.getEdgeSource(edge) );
                    Point2d target = projection.project( graph.getEdgeTarget(edge) );

                    lines.add(new LineImpl(new Point3d(source.x, source.y, 0), new Point3d(target.x, target.y, 0)));
                }
                return lines;
            }

            @Override
            public int getStrokeWidth() {
                return edgeStrokeWidth;
            }

            @Override
            public Color getColor() {
                return edgeColor;
            }

        }));

        // vertices
        group.addSubLayer(PointLayer.create(new PointElements() {



            @Override
            public Iterable<Point> getPoints() {
                Graph<V, E> graph = graphProvider.getGraph();

                LinkedList<Point> points = new LinkedList<Point>();
                for (V vertex : graph.vertexSet()) {
                    Point2d p = projection.project(vertex);
                    points.add(new PointImpl(new Point3d(p.x, p.y, 0.0)));
                }
                return points;
            }

            @Override
            public int getStrokeWidth() {
                return vertexStrokeWidth;
            }

            @Override
            public Color getColor() {
                return vertexColor;
            }

        }));

        return group;
    }


    /**
     * @deprecated use creator that uses GraphProvider instead
     */
    @Deprecated
    public static <V extends SpatialPoint,E> VisLayer create(final GraphHolder<V, E> graphHolder, final Color edgeColor, final Color vertexColor,
            final int edgeStrokeWidth, final int vertexStrokeWidth) {
        return create(graphHolder, edgeColor, vertexColor, edgeStrokeWidth, vertexStrokeWidth, 0);
    }

    /**
     * @deprecated use creator that uses GraphProvider instead
     */
    @Deprecated
    public static <V extends SpatialPoint,E> VisLayer create(final GraphHolder<V, E> graphHolder, final Color edgeColor, final Color vertexColor,
            final int edgeStrokeWidth, final int vertexStrokeWidth, final double offset) {
        GroupLayer group = GroupLayer.create();

        // edges
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                LinkedList<Line> lines = new LinkedList<Line>();
                if (graphHolder.graph != null) {
                    Vector3d transition = new Vector3d(offset, offset, 0);
                    for (E edge : graphHolder.graph.edgeSet()) {
                        Point3d source = new Point3d( graphHolder.graph.getEdgeSource(edge) );
                        source.add(transition);
                        Point3d target = new Point3d( graphHolder.graph.getEdgeTarget(edge) );
                        target.add(transition);
                        lines.add(new LineImpl(source, target));
                    }
                }
                return lines;
            }

            @Override
            public int getStrokeWidth() {
                return edgeStrokeWidth;
            }

            @Override
            public Color getColor() {
                return edgeColor;
            }

        }));

        // vertices
        group.addSubLayer(PointLayer.create(new PointElements() {

            @Override
            public Iterable<SpatialPoint> getPoints() {
                LinkedList<SpatialPoint> points = new LinkedList<SpatialPoint>();
                if (graphHolder.graph != null) {
                    for (SpatialPoint vertex : graphHolder.graph.vertexSet()) {
                        points.add(vertex);
                    }
                }
                return points;
            }

            @Override
            public int getStrokeWidth() {
                return vertexStrokeWidth;
            }

            @Override
            public Color getColor() {
                return vertexColor;
            }

        }));

        return group;
    }
}
