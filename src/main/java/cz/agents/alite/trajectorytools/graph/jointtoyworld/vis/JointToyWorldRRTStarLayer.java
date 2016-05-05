package cz.agents.alite.trajectorytools.graph.jointtoyworld.vis;

import java.awt.Color;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalState;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.planner.rrtstar.Vertex;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;

public class JointToyWorldRRTStarLayer extends AbstractLayer {


    JointToyWorldRRTStarLayer() {
    }

    private static Color getColor(double t){
        return Color.getHSBColor((new Random((int) t*1000)).nextFloat(), 0.7f, 0.7f);
    }

    public static VisLayer create(final RRTStarPlanner<JointSpatioTemporalState, JointSpatioTemporalManeuver> rrtstar, final ProjectionTo2d<TimePoint> projection, final double speed,
            final int edgeStrokeWidth, final int vertexStrokeWidth) {
        GroupLayer group = GroupLayer.create();

        // edges
        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<StyledLine> getLines() {
                LinkedList<StyledLine> lines = new LinkedList<StyledLine>();

                Queue<Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver>> queue = new LinkedList<Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver>>();
                queue.add(rrtstar.getRoot());

                while(!queue.isEmpty()) {
                    Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver> current = queue.poll();
                   for (Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver> child : current.getChildren()) {
                        queue.offer(child);

                        JointSpatioTemporalState jointSource = current.getState();
                        JointSpatioTemporalState jointTarget = child.getState();

                        // draw edge
                        if (jointSource != null && jointTarget != null) {
                            Color color = getColor(jointSource.get(0).getTime());

                            for (int i = 0; i < jointSource.nAgents(); i++) {


                                TimePoint start = jointSource.get(i);
                                TimePoint target = jointTarget.get(i);

                                double spatiallyReachedAt = start.getTime() + start.getSpatialPoint().distance(target.getSpatialPoint()) / speed;
                                TimePoint spatiallyReachedPoint = new TimePoint(jointTarget.get(i).getSpatialPoint(), spatiallyReachedAt);


                                Point2d source = projection.project(start);
                                Point2d intermediate = projection.project(spatiallyReachedPoint);
                                Point2d dest = projection.project(target);
                                lines.add(new StyledLineImpl(new Point3d(source.x, source.y, 0), new Point3d(intermediate.x, intermediate.y, 0), color, edgeStrokeWidth));
                                lines.add(new StyledLineImpl(new Point3d(intermediate.x, intermediate.y, 0), new Point3d(dest.x, dest.y, 0), color, edgeStrokeWidth));

                            }


                        }
                    }
                }

                return lines;
            }

        }));

        // vertices
        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<StyledPoint> getPoints() {
                LinkedList<StyledPoint> points = new LinkedList<StyledPoint>();

                Queue<Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver>> queue = new LinkedList<Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver>>();
                queue.add(rrtstar.getRoot());

                while(!queue.isEmpty()) {
                    Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver> current = queue.poll();
                    JointSpatioTemporalState jointState = current.getState();

                    // draw edge
                    if (jointState != null) {
                        Color color = getColor(jointState.get(0).getTime());
                        for (int i = 0; i < jointState.nAgents(); i++) {
                            Point2d point = projection.project(jointState.get(i));
                            points.add(new StyledPointImpl(new Point3d(point.x, point.y, 0), color, vertexStrokeWidth));
                        }
                    }


                    for (Vertex<JointSpatioTemporalState, JointSpatioTemporalManeuver> child : current.getChildren()) {
                        queue.offer(child);
                    }
                }
                return points;
            }
        }));

        return group;
    }

}
