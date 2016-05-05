package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.apache.log4j.Logger;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;

public class TrajectoryLayer extends CommonLayer {

    static Logger LOGGER = Logger.getLogger(TrajectoryLayer.class);

    public static interface TrajectoryProvider {
        Trajectory getTrajectory();
    }

    public static VisLayer create(final TrajectoryProvider trajectoryProvider, final ProjectionTo2d<TimePoint> projection, final Color color, final double samplingInterval, final double maxTimeArg, final char toggleKey) {
        GroupLayer group = GroupLayer.create();

        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<? extends StyledPoint> getPoints() {
                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();
                Trajectory t = trajectoryProvider.getTrajectory();

                if (t != null) {
                    double maxTime = Math.min(t.getMaxTime(), maxTimeArg);

                    Point2d start = projection.project(new TimePoint(t.getPosition(t.getMinTime()), t.getMinTime()));
                    Point2d target = projection.project(new TimePoint(t.getPosition(maxTime), maxTime));

                    points.add(new StyledPointImpl( new Point3d(start.x, start.y, 0), color, 6));
                    points.add(new StyledPointImpl( new Point3d(target.x, target.y, 0), color, 6));


                    for (double time = t.getMinTime(); time < maxTime; time += samplingInterval) {
                        OrientedPoint pos = t.getPosition(time);
                        if (pos != null) {
                            Point2d point = projection.project(new TimePoint(pos, time));
                            points.add(new StyledPointImpl(new Point3d(point.x, point.y, 0), color, 4));
                        } else {
                            throw new RuntimeException("Position for time " + time + "s is null in trajectory " + t);
                            //LOGGER.warn("Position for time " + time + "s is null in trajectory " + t);
                        }
                    }

                }

                return points;
            }

        }));

        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<? extends StyledLine> getLines() {
                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();
                return lines;
            }

        }));


        KeyToggleLayer toggle = KeyToggleLayer.create(toggleKey);
        toggle.addSubLayer(group);
        toggle.setEnabled(true);

        return toggle;

    }
}
