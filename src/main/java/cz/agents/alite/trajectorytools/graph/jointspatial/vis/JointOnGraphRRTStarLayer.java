package cz.agents.alite.trajectorytools.graph.jointspatial.vis;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;

import cz.agents.alite.trajectorytools.graph.jointspatial.JointGraphPath;
import cz.agents.alite.trajectorytools.graph.jointspatial.JointWaypointState;
import cz.agents.alite.trajectorytools.graph.spatial.vis.SpatialGraphRRTStarLayer;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.planner.rrtstar.Vertex;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.VisLayer;

public class JointOnGraphRRTStarLayer extends SpatialGraphRRTStarLayer {

    RRTStarProvider<JointWaypointState, JointGraphPath> rrtstarProvider;
    int nAgents;
    boolean[] drawAgentFlags;

    public JointOnGraphRRTStarLayer(RRTStarProvider<JointWaypointState, JointGraphPath> rrtstarProvider,
            ProjectionTo2d<TimePoint> projection,  int nAgents, double minCost, double maxCost, int vertexDotSize) {
        super(projection, minCost, maxCost, vertexDotSize);
        this.rrtstarProvider = rrtstarProvider;
        this.drawAgentFlags = new boolean[nAgents];
        this.nAgents = nAgents;
        Arrays.fill(this.drawAgentFlags, true);
    }



    public static VisLayer create(final RRTStarProvider<JointWaypointState, JointGraphPath> rrtstarProvider, final ProjectionTo2d<TimePoint> projection, int nAgents, double minCost, double maxCost, int vertexDotSize) {
        return new JointOnGraphRRTStarLayer(rrtstarProvider, projection, nAgents, minCost, maxCost, vertexDotSize);
    }



    @Override
    public void init(Vis vis) {
        super.init(vis);

        vis.addKeyListener(
        new KeyListener() {

            @Override
            public void keyTyped(KeyEvent e) {
                char c = e.getKeyChar();
                if (Character.isDigit(c) && !e.isAltDown()) {
                    int n = Integer.parseInt(new String(new char[]{c}));
                    if (n >= 0 && n < nAgents) {
                        drawAgentFlags[n] = !drawAgentFlags[n];
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

        //super.paint(canvas);

        RRTStarPlanner<JointWaypointState, JointGraphPath> rrtstar = rrtstarProvider.getRRTStar();
        if (rrtstar != null) {

            JointWaypointState lastSample = rrtstar.getLastSample();
            if (lastSample != null) {
                for (int i = 0; i < lastSample.nAgents(); i++) {
                    if (drawAgentFlags[i]){
                        if (lastSample != null) {
                            TimePoint target = new TimePoint(lastSample.getPosition(i), 0);
                            paintCircleVertex(canvas, target, Color.RED);
                            paintText(canvas, target, ""+i, Color.RED);
                        }
                    }
                }
            }

            JointWaypointState lastNewSample = rrtstar.getNewSample();
            if (lastNewSample != null) {
                for (int i = 0; i < lastNewSample.nAgents(); i++) {
                    if (drawAgentFlags[i]){
                        TimePoint target = new TimePoint(lastNewSample.getPosition(i), 0);
                        paintCircleVertex(canvas, target, Color.GREEN);
                        paintText(canvas, target, ""+i, Color.GREEN);
                    }
                }
            }



            // draw tree

            Queue<Vertex<JointWaypointState, JointGraphPath>> queue = new LinkedList<Vertex<JointWaypointState, JointGraphPath>>();
            HashMap<JointWaypointState, Double> timeAtState = new HashMap<JointWaypointState, Double>();

            queue.add(rrtstar.getRoot());
            timeAtState.put(rrtstar.getRoot().getState(), 0.0);

            JointWaypointState rootState = rrtstar.getRoot().getState();
            // draw vertex
            for (int i = 0; i < rootState.nAgents(); i++) {
                if (drawAgentFlags[i]) {
                    TimePoint target = new TimePoint(rootState.getPosition(i), 0);
                    paintVertex(canvas, target, rrtstar.getRoot().getCostFromRoot());
                }
            }



            while(!queue.isEmpty()) {
               Vertex<JointWaypointState, JointGraphPath> current = queue.poll();
               for (Vertex<JointWaypointState, JointGraphPath> child : current.getChildren()) {
                    queue.offer(child);

                    JointWaypointState jointSource = current.getState();
                    JointWaypointState jointTarget = child.getState();
                    JointGraphPath jointGraphPath = child.getEdgeFromParent();

                    double timeSource = timeAtState.get(jointSource);
                    double timeTarget = timeSource + jointGraphPath.getDuration();
                    timeAtState.put(jointTarget, timeTarget);

                    // draw edge
                    if (jointSource != null && jointTarget != null) {
                        for (int i = 0; i < jointSource.nAgents(); i++) {
                            if (drawAgentFlags[i]){

                                TimePoint start = new TimePoint(jointSource.getPosition(i), timeSource);
                                TimePoint target = new TimePoint(jointTarget.getPosition(i), timeTarget);

                                paintGraphPath(canvas,
                                        jointGraphPath.getPath(i),
                                        timeSource,
                                        current.getCostFromRoot(),
                                        child.getCostFromRoot());

                                //paintEdge(canvas, start, target, current.getCostFromRoot(), child.getCostFromRoot());
                            }
                        }
                    }

                    // draw vertex
                    for (int i = 0; i < jointTarget.nAgents(); i++) {
                        if (drawAgentFlags[i]){
                            TimePoint target = new TimePoint(jointTarget.getPosition(i), timeTarget);
                            paintVertex(canvas, target, child.getCostFromRoot());
                        }
                    }
                }
            }
        }
    }

}
