package cz.agents.alite.trajectorytools.vis;

import java.awt.Color;
import java.util.LinkedList;
import java.util.Map;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.PolygonRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.implemetation.LineImpl;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;

public class PolygonRegionLayer extends AbstractLayer {

    public static interface GraphProvider<V, E> {
    	Graph<V, E> getGraph();
    }
	
    PolygonRegionLayer() {
    }

    public static <V extends SpatialPoint,E> VisLayer create(final Map<DefaultWeightedEdge, PolygonRegion> obstacleMap, final Color color, final int strokeWidth) {
        GroupLayer group = GroupLayer.create();

        // edges
        group.addSubLayer(LineLayer.create(new LineElements() {

            @Override
            public Iterable<Line> getLines() {
                LinkedList<Line> lines = new LinkedList<Line>();
                for (PolygonRegion polygon : obstacleMap.values()) {
                	SpatialPoint[] points = polygon.getPoints();
					for (int i=0; i<points.length; i++) {
                        lines.add(new LineImpl(points[i], points[(i+1)%points.length]));
					}
                }
                return lines;
            }

            @Override
            public int getStrokeWidth() {
                return strokeWidth;
            }

            @Override
            public Color getColor() {
                return color;
            }

        }));

        return group;
    }

}