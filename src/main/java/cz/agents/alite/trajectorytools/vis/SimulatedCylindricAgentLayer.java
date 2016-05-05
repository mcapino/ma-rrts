package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.simulation.SimulatedAgentStorage;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.Circle;
import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.CircleElements;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.CircleImpl;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.CircleLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;

public class SimulatedCylindricAgentLayer extends CommonLayer {

    public static interface TimeProvider {
        double getTime();
   }

    public static VisLayer create(final SimulatedAgentStorage agentStorage, final ProjectionTo2d<TimePoint> projection, final TimeProvider timeProvider, final double radius, final double halfHeight) {
        GroupLayer group = GroupLayer.create();

        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<? extends StyledPoint> getPoints() {
                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                for (OrientedPoint pos : agentStorage.getAgents().values()) {
                    Color color = Color.RED;
                    Point2d pos2d = projection.project(new TimePoint(pos, timeProvider.getTime()));

                    points.add(new StyledPointImpl(new Point3d(pos2d.x, pos2d.y, 0), color, 12));
                }

                return points;
            }

        }));


        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<? extends StyledLine> getLines() {
                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                for (OrientedPoint center : agentStorage.getAgents().values()) {

                    lines.add(createLine(
                            new Point3d(center.x+radius, center.y, center.z+halfHeight),
                            new Point3d(center.x+radius, center.y, center.z-halfHeight)));

                    lines.add(createLine(
                            new Point3d(center.x-radius, center.y, center.z+halfHeight),
                            new Point3d(center.x-radius, center.y, center.z-halfHeight)));

                    lines.add(createLine(
                            new Point3d(center.x, center.y+radius, center.z+halfHeight),
                            new Point3d(center.x, center.y+radius, center.z-halfHeight)));

                    lines.add(createLine(
                            new Point3d(center.x, center.y-radius, center.z+halfHeight),
                            new Point3d(center.x, center.y-radius, center.z-halfHeight)));
                }

                return lines;
            }

            private StyledLine createLine(Point3d start, Point3d end) {
                Point2d start2d = projection.project(new TimePoint(start, timeProvider.getTime()));
                Point2d end2d = projection.project(new TimePoint(end, timeProvider.getTime()));
                return new StyledLineImpl(new Point3d(start2d.x, start2d.y, 0), new Point3d(end2d.x, end2d.y, 0),  Color.DARK_GRAY, 1);
            }

        }));

        group.addSubLayer(CircleLayer.create(new CircleElements() {

            @Override
            public Iterable<? extends Circle> getCircles() {
                ArrayList<Circle> circles = new ArrayList<Circle>();

                for (OrientedPoint pos : agentStorage.getAgents().values()) {
                    Point2d pos2d = projection.project(new TimePoint(pos, timeProvider.getTime()));
                    circles.add(new CircleImpl(new Point3d(pos2d.x, pos2d.y, 0), radius));
                }

                return circles;
            }

            @Override
            public Color getColor() {
                return Color.DARK_GRAY;
            }

            @Override
            public int getStrokeWidth() {
                return 1;
            }

        }));

        group.addSubLayer(AgentIdLayer.create(agentStorage, Color.BLACK, 1, "a"));
        return group;
    }
}
