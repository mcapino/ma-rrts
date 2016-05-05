package cz.agents.alite.trajectorytools.graph.jointtoyworld.vis;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;

public class MissionsLayer extends AbstractLayer {

    public static interface MissionsProvider {
        public SpatialPoint[] getStarts();
        public SpatialPoint[] getTargets();
    }

    MissionsLayer() {
    }

    public static VisLayer create(final MissionsProvider missionsProvider, final ProjectionTo2d<SpatialPoint> projection,  final Color color) {
        GroupLayer group = GroupLayer.create();


        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<? extends StyledPoint> getPoints() {
                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                for(SpatialPoint start : missionsProvider.getStarts()){
                    Point2d p = projection.project(start);

                    points.add(new StyledPointImpl(new Point3d(p.x, p.y, 0), color, 6));
                }

                for(SpatialPoint target : missionsProvider.getTargets()){
                    Point2d p = projection.project(target);

                    points.add(new StyledPointImpl(new Point3d(p.x, p.y, 0), color, 6));
                }

                return points;
            }

        }));


      //draw  info
        group.addSubLayer(new CommonLayer() {

            @Override
            public void paint(Graphics2D canvas) {
                super.paint(canvas);

                canvas.setColor(color);
                int i=0;
                for(SpatialPoint start : missionsProvider.getStarts()){
                    Point2d p = projection.project(start);
                    canvas.drawString("s"+i, (Vis.transX(p.x) + 5), (Vis.transY(p.y) + 5));
                    i++;
                }

                i = 0;
                for(SpatialPoint target : missionsProvider.getTargets()){
                    Point2d p = projection.project(target);
                    canvas.drawString("d"+i, (Vis.transX(p.x) + 5), (Vis.transY(p.y) + 20));
                    i++;
                }
            }
        });

        group.setHelpOverrideString("Shows the start and target position of each agent.");

        return group;


    }
}
