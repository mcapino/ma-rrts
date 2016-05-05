package cz.agents.alite.trajectorytools.graph.spatial.vis;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import javax.vecmath.Point2d;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.planner.rrtstar.Vertex;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.deconfliction.util.ColorMap;

public class SpatialGraphRRTStarLayer extends AbstractLayer {

    static public abstract class RRTStarProvider<V,E> {
        public abstract RRTStarPlanner<V, E> getRRTStar();
    }

    private RRTStarProvider<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstarProvider;
    protected ProjectionTo2d<TimePoint> projection;
    protected int vertexDotSize;
    protected double minCost;
    protected double maxCost;
    protected ColorMap colorMap;

    public SpatialGraphRRTStarLayer(RRTStarProvider<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstarProvider,
            ProjectionTo2d<TimePoint> projection,  double minCost, double maxCost, int vertexDotSize) {
        this(projection, minCost, maxCost, vertexDotSize);
        this.rrtstarProvider = rrtstarProvider;
    }

    protected SpatialGraphRRTStarLayer(ProjectionTo2d<TimePoint> projection, double minCost, double maxCost, int vertexDotSize) {
        super();
        this.projection = projection;
        this.vertexDotSize = vertexDotSize;
        this.minCost = minCost;
        this.maxCost = maxCost;
        this.colorMap = new ColorMap(minCost, maxCost, ColorMap.JET);
    }

    public static VisLayer create(
            final RRTStarProvider<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstarProvider,
            final ProjectionTo2d<TimePoint> projection,
            double minCost, double maxCost, int vertexDotSize) {
        return new SpatialGraphRRTStarLayer(rrtstarProvider, projection,
                minCost, maxCost, vertexDotSize);
    }

    @Override
    public void init(Vis vis) {
        super.init(vis);

        vis.addKeyListener(
        new KeyListener() {

            @Override
            public void keyTyped(KeyEvent e) {
            }

            @Override
            public void keyReleased(KeyEvent e) {
            }

            @Override
            public void keyPressed(KeyEvent e) {
                String text = KeyEvent.getKeyText(e.getKeyCode());
            }
        });
    }



    @Override
    public void paint(Graphics2D canvas) {

        super.paint(canvas);

        RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstar = rrtstarProvider.getRRTStar();
        if (rrtstar != null) {
            paintTree(canvas, rrtstar);
        }
    }

    public void paintTree(
            Graphics2D canvas,
            RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrtstar) {

        Waypoint lastSample = rrtstar.getLastSample();
        if (lastSample != null) {
            TimePoint target = new TimePoint(lastSample, 0);
            paintCircleVertex(canvas, target, Color.RED);
            paintText(canvas, target, "", Color.RED);
        }

        Waypoint lastNewSample = rrtstar.getNewSample();
        if (lastNewSample != null) {
            TimePoint target = new TimePoint(lastNewSample, 0);
            paintCircleVertex(canvas, target, Color.GREEN);
            paintText(canvas, target, "", Color.GREEN);
        }

        // draw tree

        Queue<Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>>> queue = new LinkedList<Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>>>();
        queue.add(rrtstar.getRoot());

        Waypoint rootState = rrtstar.getRoot().getState();
        // draw vertex
        TimePoint target = new TimePoint(rootState, 0);
        paintVertex(canvas, target, rrtstar.getRoot().getCostFromRoot());

        while (!queue.isEmpty()) {
            Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> current = queue
                    .poll();
            for (Vertex<Waypoint, GraphPath<Waypoint, SpatialManeuver>> child : current
                    .getChildren()) {
                queue.offer(child);

                Waypoint sourceWaypoint = current.getState();
                Waypoint targetWaypoint = child.getState();
                GraphPath<Waypoint, SpatialManeuver> jointGraphPath = child
                        .getEdgeFromParent();

                // draw edge
                if (sourceWaypoint != null && targetWaypoint != null) {
                    TimePoint startTimePoint = new TimePoint(sourceWaypoint,
                            current.getCostFromRoot());
                    TimePoint targetTimePoint = new TimePoint(targetWaypoint,
                            child.getCostFromRoot());

                    paintGraphPath(canvas, jointGraphPath,
                            current.getCostFromRoot(),
                            current.getCostFromRoot(), child.getCostFromRoot());

                    // paintEdge(canvas, start, target,
                    // current.getCostFromRoot(), child.getCostFromRoot());
                }

                // draw vertex
                TimePoint targetTimePoint = new TimePoint(targetWaypoint,
                        child.getCostFromRoot());
                paintVertex(canvas, targetTimePoint, child.getCostFromRoot());
            }
        }

    }

    protected void paintVertex(Graphics2D canvas, TimePoint point, double cost) {
        Point2d point2d = projection.project(point);
        canvas.setColor(colorMap.getColor(cost));
        canvas.fillOval(Vis.transX(point2d.x)-vertexDotSize, Vis.transY(point2d.y)-vertexDotSize, vertexDotSize*2, vertexDotSize*2);

    }

    protected void paintCircleVertex(Graphics2D canvas, TimePoint point, Color color) {
        Point2d point2d = projection.project(point);
        canvas.setColor(color);
        canvas.drawOval(Vis.transX(point2d.x)-(vertexDotSize+2), Vis.transY(point2d.y)-(vertexDotSize+2), (vertexDotSize+2)*2, (vertexDotSize+2)*2);
    }


    protected void paintEdge(Graphics2D canvas, TimePoint start, TimePoint target, double startCost, double endCost) {
        Point2d source = projection.project(start);
        Point2d dest = projection.project(target);
        canvas.setColor(colorMap.getColor(startCost));
        canvas.drawLine(Vis.transX(source.x), Vis.transY(source.y), Vis.transX(dest.x), Vis.transY(dest.y));
    }

    protected void paintText(Graphics2D canvas, TimePoint point, String text, Color color) {
        Point2d p = projection.project(point);
        canvas.setColor(color);
        canvas.drawString(text, (Vis.transX(p.x) + 8), (Vis.transY(p.y) + 5));
    }

    protected void paintGraphPath(Graphics2D canvas, GraphPath<Waypoint, SpatialManeuver> graphPath, double startTime, double startCost, double endCost) {
        Graph<Waypoint, SpatialManeuver> graph = graphPath.getGraph();
        List<SpatialManeuver> edges = graphPath.getEdgeList();

        double time = startTime;
        double cost = startCost;
        for (SpatialManeuver edge : edges) {
            Waypoint start = graph.getEdgeSource(edge);
            Waypoint target = graph.getEdgeTarget(edge);
            canvas.setColor(colorMap.getColor(cost));


            Point2d source = projection.project(new TimePoint(start, time));
            Point2d dest = projection.project(new TimePoint(target, time+1.0));

            cost += graph.getEdgeWeight(edge) * 4;
            time++;

            canvas.drawLine(Vis.transX(source.x), Vis.transY(source.y), Vis.transX(dest.x), Vis.transY(dest.y));
        }
    }
}
