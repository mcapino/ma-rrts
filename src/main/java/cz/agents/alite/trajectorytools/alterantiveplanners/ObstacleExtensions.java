package cz.agents.alite.trajectorytools.alterantiveplanners;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Set;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.jgrapht.Graph;

import cz.agents.alite.planner.spatialmaneuver.zone.BoxZone;
import cz.agents.alite.planner.spatialmaneuver.zone.TransformZone;
import cz.agents.alite.planner.spatialmaneuver.zone.Zone;
import cz.agents.alite.trajectorytools.graph.spatial.GraphWithObstacles;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.planner.PathPlanner;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ObstacleExtensions<V extends SpatialPoint, E> implements AlternativePathPlanner<V,E> {

    private static final int DIRECTIONS = 4;

    private final PathPlanner<V, E> planner;

    public ObstacleExtensions(PathPlanner<V, E> planner) {
        this.planner = planner;
    }

    final Set<PlannedPath<V, E>> paths = new HashSet<PlannedPath<V, E>>();

    @Override
    public Collection<PlannedPath<V, E>> planPath(
            GraphWithObstacles<V, E> originalGraph,
            V startVertex, V endVertex) {

        final Set<PlannedPath<V, E>> paths = new HashSet<PlannedPath<V, E>>();

        if (originalGraph.getObstacles().size() > 6) {
            return paths;
        }

        ObstacleExtender<V,E> obstacleExtender = new ObstacleExtender<V,E>(originalGraph);

        for (Graph<V, E> graph : obstacleExtender) {
            PlannedPath<V, E> path = planner.planPath(graph, startVertex, endVertex);

            if ( path != null ) {
                paths.add( path );
            }
        }

        return paths;
    }

    static class ObstacleExtender<VV extends SpatialPoint, EE> implements Iterable<Graph<VV, EE>>{

        int[] directions;
        private final GraphWithObstacles<VV, EE> originalGraph;

        public ObstacleExtender(GraphWithObstacles<VV, EE> originalGraph) {
            this.originalGraph = originalGraph;

            directions = new int[originalGraph.getObstacles().size()];
        }

        @Override
        public Iterator<Graph<VV, EE>> iterator() {
            return new Iterator<Graph<VV, EE>>() {

                boolean firstCall = true;

                @Override
                public boolean hasNext() {
                    for (int i = directions.length-1; i >= 0; i--) {
                        if ( directions[i] != DIRECTIONS-1 ) {
                            return true;
                        }
                    }
                    return false;
                }

                @Override
                public Graph<VV, EE> next() {
                    if ( !hasNext() ) {
                        throw new NoSuchElementException("No next element!");
                    }

                    if ( !firstCall ) {
                        incrementDirections();
                    }
                    firstCall = false;

                    return generateNextGraph();
                }

                protected Graph<VV, EE> generateNextGraph() {
                   Graph<VV, EE>graph = SpatialGraphs.clone(originalGraph);

                    int currObstacle = 0;

                    for (SpatialPoint obstacle : originalGraph.getObstacles()) {
                        removeObstacleExtension(graph, obstacle, directions[ currObstacle++ ] );
                    }

                    return graph;

                }

                private void removeObstacleExtension(Graph<VV, EE> graph, SpatialPoint obstacle, int direction) {

                    Zone zone = createZone(obstacle, direction);

                    for (VV vertex : new ArrayList<VV>(graph.vertexSet())) {
                        if ( zone.testPoint( vertex ) ) {
                            graph.removeVertex( vertex );
                        }
                    }

                    for (EE edge : new ArrayList<EE>(graph.edgeSet())) {
                        if ( zone.testLine(graph.getEdgeSource(edge), graph.getEdgeTarget(edge), null) ) {
                            graph.removeEdge(edge);
                        }
                    }
                }

                private Zone createZone(SpatialPoint obstacle, int direction) {
                    Zone zone;
                    Zone boxZone = new BoxZone(new Vector3d(Double.MAX_VALUE, 0.2, 0.2));

                    Vector3d translation = new Vector3d(obstacle.x, obstacle.y, obstacle.z - 0.1);
                    switch (direction) {
                    case 0: // NORTH
                        translation.x -= 0.1;
                        zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), -Math.PI/2);
                        break;
                    case 1: // EAST
                        translation.y -= 0.1;
                        zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), 0);
                        break;
                    case 2: // SOUTH
                        translation.x +=0.1;
                        zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), Math.PI/2);
                        break;
                    case 3: // WEST
                        translation.y += 0.1;
                        zone = new TransformZone(boxZone, translation, new Vector2d(1, 1), Math.PI);
                        break;
                    default:
                        throw new IllegalArgumentException("Unknown direction: " + direction);
                    }

                    return zone;
                }

                @Override
                public void remove() {
                    throw new UnsupportedOperationException();
                }
            };
        }

        private void incrementDirections() {
            directions[0] ++;
            int index = 0;
            while ( directions[index] >= DIRECTIONS ) {
                directions[index] = 0;
                index ++;
                directions[index] ++;
            }
        }
    }

    @Override
    public String getName() {
        return "Obstacle Extension";
    }
}
