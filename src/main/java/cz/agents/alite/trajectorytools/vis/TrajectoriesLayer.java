package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.apache.log4j.Logger;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.TrajectoryLayer.TrajectoryProvider;
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

public class TrajectoriesLayer extends CommonLayer {

    static Logger LOGGER = Logger.getLogger(TrajectoriesLayer.class);

    public static interface TrajectoriesProvider {
        Trajectory[] getTrajectories();
    }

    public static VisLayer create(final TrajectoriesProvider trajectoriesProvider, final ProjectionTo2d<TimePoint> projection, final double samplingInterval, final double maxTimeArg, final char toggleKey) {
        GroupLayer group = GroupLayer.create();
        
        Trajectory[] trajectories = trajectoriesProvider.getTrajectories();
        
        for (int i=0; i<trajectories.length; i++) {
        	final int agentNo = i;
        	group.addSubLayer(TrajectoryLayer.create(new TrajectoryProvider() {
				
				@Override
				public Trajectory getTrajectory() {
					return trajectoriesProvider.getTrajectories()[agentNo];
				}
			}, projection, AgentColors.getColorForAgent(agentNo), samplingInterval, maxTimeArg, toggleKey));
        }

        return group;

    }
}
