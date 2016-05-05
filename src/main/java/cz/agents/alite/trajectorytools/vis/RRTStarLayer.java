package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector4d;

import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.planner.rrtstar.Vertex;
import cz.agents.alite.trajectorytools.util.OrientedTimePoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.Circle;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.Point;
import cz.agents.alite.vis.element.aggregation.CircleElements;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.aggregation.PointElements;
import cz.agents.alite.vis.element.implemetation.CircleImpl;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.element.implemetation.PointImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.CircleLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;
import cz.agents.alite.vis.layer.terminal.PointLayer;

public class RRTStarLayer extends AbstractLayer {


    RRTStarLayer() {
    }
    
	public static <V> VisLayer create(final RRTStarPlanner<V, ?> rrtstar,
			final ProjectionTo2d<? super V> projection, final Color edgeColor,
			final Color vertexColor, final int edgeStrokeWidth,
			final int vertexStrokeWidth) {
		return create(rrtstar, projection, edgeColor, vertexColor, edgeStrokeWidth, vertexStrokeWidth, false);
	}

	public static <V> VisLayer create(final RRTStarPlanner<V, ?> rrtstar,
			final ProjectionTo2d<? super V> projection, final Color edgeColor,
			final Color vertexColor, final int edgeStrokeWidth,
			final int vertexStrokeWidth, final boolean visualizeSearch) {
        GroupLayer group = GroupLayer.create();

        // edges
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                LinkedList<Line> lines = new LinkedList<Line>();

                Queue<Vertex<V,?>> queue = new LinkedList<Vertex<V,?>>();
                queue.add(rrtstar.getRoot());

                while(!queue.isEmpty()) {
                   Vertex<V,?> current = queue.poll();
                   for (Vertex<V,?> child : current.getChildren()) {
                        queue.offer(child);

                        Point2d source = projection.project(current.getState());
                        Point2d target = projection.project(child.getState());

                        // draw edge
                        if (source != null && target != null) {
                            lines.add(new LineImpl(new Point3d(source.x, source.y, 0), new Point3d(target.x, target.y, 0)));
                        }
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
            public Iterable<Point> getPoints() {
                LinkedList<Point> points = new LinkedList<Point>();

                Queue<Vertex<V,?>> queue = new LinkedList<Vertex<V,?>>();
                queue.add(rrtstar.getRoot());

                while(!queue.isEmpty()) {
                    Vertex<V,?> current = queue.poll();


                    Point2d point = projection.project(current.getState());
                    points.add(new PointImpl(new Point3d(point.x, point.y, 0)));

                    for (Vertex<V,?> child : current.getChildren()) {
                        queue.offer(child);
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

        // vertices (orientation) HACK!!!
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                LinkedList<Line> lines = new LinkedList<Line>();

                Queue<Vertex<V,?>> queue = new LinkedList<Vertex<V,?>>();
                queue.add(rrtstar.getRoot());

                while(!queue.isEmpty()) {
                    Vertex<V,?> current = queue.poll();

                    if (current.getState() instanceof OrientedTimePoint) {
                        OrientedTimePoint point = (OrientedTimePoint) current.getState();

                        Point2d start = projection.project((V) point);
                        TimePoint endTimePoint = new TimePoint(point);
                        
                        Vector4d vector = new Vector4d(point.orientation.x, point.orientation.y, point.orientation.z, 1);
                        vector.scale(10);
                        
                        endTimePoint.add(vector);
                        Point2d end = projection.project((V) endTimePoint);
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                    }

                    for (Vertex<V,?> child : current.getChildren()) {
                        queue.offer(child);
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
                return Color.BLUE;
            }

        }));
        
        // random sample
        group.addSubLayer(PointLayer.create(new PointElements() {

            @Override
            public Iterable<Point> getPoints() {
            	LinkedList<Point> points = new LinkedList<Point>();

            	if (!visualizeSearch) return points;
            	
                V lastSample = rrtstar.getLastSample();

                if (lastSample != null) {
                	Point2d point = projection.project(lastSample);
                	points.add(new PointImpl(new Point3d(point.x, point.y, 0)));
                }
                return points;
            }

            @Override
            public int getStrokeWidth() {
                return 10;
            }

            @Override
            public Color getColor() {
                return Color.RED;
            }

        }));
        
        // new sample
        group.addSubLayer(PointLayer.create(new PointElements() {

            @Override
            public Iterable<Point> getPoints() {
                LinkedList<Point> points = new LinkedList<Point>();
                
                if (!visualizeSearch) return points;
                
                V newSample = rrtstar.getNewSample();

                if (newSample != null) {
                	Point2d point = projection.project(newSample);
                	points.add(new PointImpl(new Point3d(point.x, point.y, 0)));
                }
                return points;
            }

            @Override
            public int getStrokeWidth() {
                return 6;
            }

            @Override
            public Color getColor() {
                return Color.MAGENTA;
            }

        }));

        group.addSubLayer(CircleLayer.create(new CircleElements() {
			
			@Override
			public int getStrokeWidth() {
				return 1;
			}
			
			@Override
			public Color getColor() {
				return Color.MAGENTA;
			}
			
			@Override
			public Iterable<? extends Circle> getCircles() {
				ArrayList<Circle> circles = new ArrayList<Circle>();
				
				if (!visualizeSearch) return circles;
				
				V newSample = rrtstar.getNewSample();

	            if (newSample != null) {
	            	Point2d point = projection.project(newSample);
	                circles.add(new CircleImpl(new Point3d(point.x, point.y, 0), rrtstar.getNearBallRadius()));
	            }
				
				return circles;
			}
		}));
        
        return group;
    }
}
