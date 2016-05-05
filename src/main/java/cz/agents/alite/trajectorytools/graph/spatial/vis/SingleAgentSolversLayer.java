package cz.agents.alite.trajectorytools.graph.spatial.vis;

import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point2d;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.deconfliction.solver.central.SingleAgentRRTStarSolver;

public class SingleAgentSolversLayer extends SpatialGraphRRTStarLayer {

    boolean paintAgents[] = new boolean[1000];

    public interface SingleAgentSolversProvider {
        SingleAgentRRTStarSolver[] getSolvers();
    }

    private SingleAgentSolversProvider solversProvider;

    public static VisLayer create(
            final SingleAgentSolversProvider solversProvider,
            final ProjectionTo2d<TimePoint> projection,
            double minCost, double maxCost, int vertexDotSize) {
        return new SingleAgentSolversLayer(solversProvider, projection,
                minCost, maxCost, vertexDotSize);

    }

    public SingleAgentSolversLayer(SingleAgentSolversProvider solversProvider, ProjectionTo2d<TimePoint> projection,
            double minCost, double maxCost, int vertexDotSize) {
        super(projection, minCost, maxCost, vertexDotSize);
        this.solversProvider = solversProvider;
        Arrays.fill(paintAgents, true);
    }

    @Override
    public void init(Vis vis) {
        super.init(vis);
        vis.addKeyListener(
                new KeyListener() {

                    @Override
                    public void keyTyped(KeyEvent e) {
                        if (e.isAltDown()) {
                            char c = e.getKeyChar();
                            if (Character.isDigit(c)) {
                                int i = Integer.parseInt(""+c);
                                paintAgents[i] = !paintAgents[i];
                            }
                        }

                    }

                    @Override
                    public void keyReleased(KeyEvent e) {
                    }

                    @Override
                    public void keyPressed(KeyEvent e) {
                    }
                });
    }

    @Override
    public void paint(Graphics2D canvas) {
        SingleAgentRRTStarSolver[] solvers = solversProvider.getSolvers();

        if (solvers != null) {
            for (int i=0; i<solvers.length;i++) {

                RRTStarPlanner<Waypoint, GraphPath<Waypoint, SpatialManeuver>> rrt
                    = solvers[i].getRRTStarPlanner();
                if (rrt != null && paintAgents[i]) {
                    paintTree(canvas, rrt);
                }
            }
        }
    }

    @Override
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

            cost += graph.getEdgeWeight(edge);
            time++;

            canvas.drawLine(Vis.transX(source.x), Vis.transY(source.y), Vis.transX(dest.x), Vis.transY(dest.y));
        }
    }
}
