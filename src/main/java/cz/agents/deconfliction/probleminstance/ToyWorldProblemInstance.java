package cz.agents.deconfliction.probleminstance;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Random;

import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.graph.spatial.region.BoxRegion;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public abstract class ToyWorldProblemInstance {
    protected int cols;
    protected int rows;

    protected Collection<SpaceRegion> obstacles = new LinkedList<SpaceRegion>();
    protected int obstructedCells;

    protected Random random;
    protected BoxRegion bounds3d;
    
    protected double agentSizeRadius;

    DirectedGraph<Waypoint, SpatialManeuver> graph;
    
    static class CannotPlaceObstaclesException extends RuntimeException {};

    public ToyWorldProblemInstance(int cols, int rows, double obstaclesRatio, double agentSizeRadius, int seed) {
        super();
        this.cols = cols;
        this.rows = rows;
        this.random = new Random(seed);
        this.agentSizeRadius = agentSizeRadius;

        bounds3d = new BoxRegion(new SpatialPoint(-0.5, -0.5, 0), new SpatialPoint(cols-0.5, rows-0.5, 0));

        generateObstacles(obstaclesRatio);

        graph = generateManeuverGraph(cols, rows, obstacles);
    }

    public ToyWorldProblemInstance(int cols, int rows, double obstacledRatio, double obstacleSide, double agentSizeRadius, int seed) {
        super();
        this.cols = cols;
        this.rows = rows;
        this.random = new Random(seed);
        this.agentSizeRadius = agentSizeRadius;

        bounds3d = new BoxRegion(new SpatialPoint(-0.5, -0.5, -0.5), new SpatialPoint(cols-0.5, rows-0.5, 0.5));

        generateObstacles(obstacledRatio, obstacleSide);

        graph = generateManeuverGraph(cols, rows, obstacles);
    }

    private static DirectedGraph<Waypoint, SpatialManeuver> generateManeuverGraph(int cols, int rows, Collection<SpaceRegion> obstacles) {
        DirectedGraph<Waypoint, SpatialManeuver> graph = SpatialGridFactory.create4WayUnitStepGridAsDirectedGraph(cols, rows);
        SpatialGraphs.cutOutObstacles(graph, obstacles);
        return graph;
    }

    private void generateObstacles(double obstaclesRatio) {
        double spaceObstructed = 0.0;
        final double space = cols * rows;
        final double obstacleSide = 4.8;

        while (spaceObstructed/space < obstaclesRatio) {
            // add new obstacle
            SpatialPoint center = sampleFreeSpace();

            SpaceRegion obstacle = new BoxRegion(
                    new SpatialPoint(center.x - obstacleSide/2, center.y - obstacleSide/2, -0.1),
                    new SpatialPoint(center.x + obstacleSide/2, center.y + obstacleSide/2, 0.1));

            spaceObstructed += obstacleSide * obstacleSide;
            obstacles.add(obstacle);
        }
    }

    private void generateObstacles(double obstacleRatio, double obstacleSide) {

        assert(obstacleRatio < 1.0);

        int totalCells = cols*rows;
        int goalObstructedCells = (int) Math.round(obstacleRatio*totalCells);
        int totalTries = 0;

        while (obstructedCells < goalObstructedCells) {
            if (obstructedCells(obstacleSide, 1.0) <= (goalObstructedCells - obstructedCells)) {

                BoxRegion obstacle = null;
                int tries=0;
                do {
                    SpatialPoint center = sampleFreeSpace();

                    obstacle = new BoxRegion(
                            new SpatialPoint(center.x - obstacleSide/2, center.y - obstacleSide/2, 0),
                            new SpatialPoint(center.x + obstacleSide/2, center.y + obstacleSide/2, 0));

                    tries++;

                } while ((!isInFreeSpace(obstacle) || !bounds3d.isInside(obstacle)) && tries < 1000);

                if (isInFreeSpace(obstacle) && bounds3d.isInside(obstacle)) {
                    obstacles.add(obstacle);
                    obstructedCells += obstructedCells(obstacleSide, 1.0);
                } else {
                    // Try again with smaller obstacle
                    totalTries += tries;

                    if (totalTries > 5000) {
                        throw new CannotPlaceObstaclesException();
                    } else {
                        if (obstacleSide > 2.0) {
                            obstacleSide -= 2.0;
                        }
                    }
                }

            } else {
                if (obstacleSide > 2.0) {
                    obstacleSide -= 2.0;
                }
            }
        }
    }

    private int obstructedCells(double side, double cellSide) {
        return (int) Math.pow( 2 * Math.floor((side/2)/cellSide) + 1, 2);
    }

    private boolean isInFreeSpace(BoxRegion box){
        // corner 1 .. corner 8
        SpatialPoint c1 = new SpatialPoint(box.getCorner1().x, box.getCorner1().y, box.getCorner1().z);
        SpatialPoint c2 = new SpatialPoint(box.getCorner1().x, box.getCorner1().y, box.getCorner2().z);
        SpatialPoint c3 = new SpatialPoint(box.getCorner2().x, box.getCorner1().y, box.getCorner2().z);
        SpatialPoint c4 = new SpatialPoint(box.getCorner2().x, box.getCorner2().y, box.getCorner2().z);
        SpatialPoint c5 = new SpatialPoint(box.getCorner1().x, box.getCorner2().y, box.getCorner2().z);
        SpatialPoint c6 = new SpatialPoint(box.getCorner1().x, box.getCorner2().y, box.getCorner1().z);
        SpatialPoint c7 = new SpatialPoint(box.getCorner2().x, box.getCorner2().y, box.getCorner1().z);
        SpatialPoint c8 = new SpatialPoint(box.getCorner2().x, box.getCorner1().y, box.getCorner1().z);

        return isInFreeSpace(c1) && isInFreeSpace(c2) && isInFreeSpace(c3) && isInFreeSpace(c4)
                && isInFreeSpace(c5) && isInFreeSpace(c6) && isInFreeSpace(c7) && isInFreeSpace(c8);
    }



    public boolean isInFreeSpace(SpatialPoint point) {
        for (SpaceRegion obstacle : obstacles) {
            if (obstacle.isInside(point)) {
                return false;
            }
        }

        return true;
    }

    public SpatialPoint sampleFreeSpace() {

        SpatialPoint point;
        do {
            int x = random.nextInt(cols);
            int y = random.nextInt(rows);

            point = new SpatialPoint(x, y, 0);

        } while (!isInFreeSpace(point));

        return point;
    }

    public SpatialPoint sampleFreeSpace(double obstacleSide) {

        SpatialPoint point;
        do {
            double x = obstacleSide/2-0.5 + random.nextDouble()*(cols-obstacleSide) ;
            double y = obstacleSide/2-0.5 + random.nextDouble()*(rows-obstacleSide) ;

            point = new SpatialPoint(x, y, 0);

        } while (!isInFreeSpace(point));

        return point;
    }


    public Collection<SpaceRegion> getObstacles() {
        return obstacles;
    }

    public double getCellSize() {
        return 1.0;
    }

    public DirectedGraph<Waypoint, SpatialManeuver> getGraph() {
        return graph;
    }

    public double getAgentSizeRadius() {
        return agentSizeRadius;
    }

    public BoxRegion getBounds3d() {
        return bounds3d;
    }
    
    public double getAverageObstacleSize() {
    	if (!obstacles.isEmpty()) {
    		return obstructedCells / obstacles.size();
    	} else {
    		return 0;
    	}
    }

}
