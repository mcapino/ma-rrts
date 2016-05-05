package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.Collection;
import java.util.LinkedList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.Box4dRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.MovingSphereSafeRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.StaticBoxRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.StaticSphereRegion;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.Circle;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.CircleElements;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.CircleImpl;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.CircleLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;

public class RegionsLayer extends AbstractLayer {

    public static class RegionsProvider {
        public Collection<SpaceRegion> getSpaceRegions() { return new LinkedList<SpaceRegion>(); };
        public Collection<SpaceTimeRegion> getSpaceTimeRegions() { return new LinkedList<SpaceTimeRegion>(); };
    }

    RegionsLayer() {
    }

    public static VisLayer create(final RegionsProvider regionsProvider,
            final ProjectionTo2d<TimePoint> projection, final Color edgeColor,
            final int edgeStrokeWidth) {
        GroupLayer group = GroupLayer.create();

        // boxes
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                Collection<SpaceTimeRegion> regions4d = regionsProvider.getSpaceTimeRegions();
                Collection<SpaceRegion> regions3d = regionsProvider.getSpaceRegions();

                LinkedList<Line> lines = new LinkedList<Line>();

                for (SpaceRegion region : regions3d) {
                    if (region instanceof BoxRegion) {
                        BoxRegion box = (BoxRegion) region;

                        Point2d corner1 = projection.project(new TimePoint(box.getCorner1(), 0));
                        Point2d corner2 = projection.project(new TimePoint(box.getCorner2(), Double.POSITIVE_INFINITY ));

                        double x1 = corner1.x;
                        double y1 = corner1.y;

                        double x2 = corner2.x;
                        double y2 = corner2.y;

                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x1,y2,0)));
                        lines.add(new LineImpl(new Point3d(x2,y1,0), new Point3d(x2,y2,0)));
                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x2,y1,0)));
                        lines.add(new LineImpl(new Point3d(x1,y2,0), new Point3d(x2,y2,0)));
                    }
                }

                for (SpaceTimeRegion region : regions4d) {
                    if (region instanceof StaticBoxRegion) {
                        StaticBoxRegion box = (StaticBoxRegion) region;

                        Point2d corner1 = projection.project(new TimePoint(box.getCorner1(), 0));
                        Point2d corner2 = projection.project(new TimePoint(box.getCorner2(), Double.POSITIVE_INFINITY ));

                        double x1 = corner1.x;
                        double y1 = corner1.y;

                        double x2 = corner2.x;
                        double y2 = corner2.y;

                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x1,y2,0)));
                        lines.add(new LineImpl(new Point3d(x2,y1,0), new Point3d(x2,y2,0)));
                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x2,y1,0)));
                        lines.add(new LineImpl(new Point3d(x1,y2,0), new Point3d(x2,y2,0)));

                    }

                    if (region instanceof Box4dRegion) {
                        Box4dRegion box = (Box4dRegion) region;

                        Point2d corner1 = projection.project(new TimePoint(box.getCorner1()));
                        Point2d corner2 = projection.project(new TimePoint(box.getCorner2()));


                        double x1 = corner1.x;
                        double y1 = corner1.y;

                        double x2 = corner2.x;
                        double y2 = corner2.y;

                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x1,y2,0)));
                        lines.add(new LineImpl(new Point3d(x2,y1,0), new Point3d(x2,y2,0)));
                        lines.add(new LineImpl(new Point3d(x1,y1,0), new Point3d(x2,y1,0)));
                        lines.add(new LineImpl(new Point3d(x1,y2,0), new Point3d(x2,y2,0)));
                    }

                    if (region instanceof StaticSphereRegion) {
                        StaticSphereRegion sphere = (StaticSphereRegion) region;
                        Point3d c = sphere.getCenter();
                        double r =  sphere.getRadius();

                        Point2d start;
                        Point2d end;

                        start = projection.project(new TimePoint(c.x+r, c.y, c.z, 0));
                        end = projection.project(new TimePoint(c.x+r, c.y, c.z, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                        start = projection.project(new TimePoint(c.x-r, c.y, c.z, 0));
                        end = projection.project(new TimePoint(c.x-r, c.y, c.z, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                        start = projection.project(new TimePoint(c.x, c.y+r, c.z, 0));
                        end = projection.project(new TimePoint(c.x, c.y+r, c.z, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                        start = projection.project(new TimePoint(c.x, c.y-r, c.z, 0));
                        end = projection.project(new TimePoint(c.x, c.y-r, c.z, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                        start = projection.project(new TimePoint(c.x, c.y, c.z+r, 0));
                        end = projection.project(new TimePoint(c.x, c.y, c.z+r, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));

                        start = projection.project(new TimePoint(c.x, c.y, c.z-r, 0));
                        end = projection.project(new TimePoint(c.x, c.y, c.z-r, Double.POSITIVE_INFINITY));
                        lines.add(new LineImpl(new Point3d(start.x, start.y, 0), new Point3d(end.x, end.y, 0)));
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

        // circles
        group.addSubLayer(CircleLayer.create(new CircleElements() {

            @Override
            public Iterable<Circle> getCircles() {
                Collection<SpaceTimeRegion> regions = regionsProvider.getSpaceTimeRegions();
                LinkedList<Circle> circles = new LinkedList<Circle>();

                for (SpaceTimeRegion region : regions) {
                    if (region instanceof StaticSphereRegion) {
                        StaticSphereRegion sphere = (StaticSphereRegion) region;
                        Point2d center1 = projection.project(new TimePoint(sphere.getCenter().x,sphere.getCenter().y,sphere.getCenter().z,0.0));
                        circles.add(new CircleImpl(new Point3d(center1.x, center1.y, 0), sphere.getRadius()));
                    }

                    if (region instanceof MovingSphereSafeRegion) {
                        MovingSphereSafeRegion safeRegion = (MovingSphereSafeRegion) region;
                        Trajectory traj = safeRegion.getTrajectory();

                        // Not finished yet...
                    }
                }

                return circles;
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


        return group;
    }
}
