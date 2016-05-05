package cz.agents.deconfliction.probleminstance;

import java.util.Random;

import org.jgrapht.DirectedGraph;
import org.jgrapht.alg.ConnectivityInspector;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.probleminstance.ToyWorldMultiAgentProblemInstance.CannotPlaceAgentsException;
import cz.agents.deconfliction.probleminstance.ToyWorldProblemInstance.CannotPlaceObstaclesException;

public class ProblemInstanceSetGenerator {

    static Random random = new Random(1);

    static String scenario = "superconflict";
    static double obstacleratio = 0.0;
    static double separations[] =  {0.8};
    static int gridsizes[] = {10, 30, 50, 70, 90};
    static int nagentss[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    static double obstaclesizes[] = {0.8};
    static int n = 5;
    static String algorithms[] = {"ja", "oa", "marrt", "bmarrt"};

    public static String generate() {
        int instanceNo = 1;
        StringBuilder sb = new StringBuilder();
        for (int gridsize : gridsizes) {
            for (double obstaclesize : obstaclesizes) {
                for (double separation : separations) {
                    for (int nagents : nagentss) {

                        // Check if we can fit the agents to the space
                        if ((nagents * (separation) * (separation)) > 0.8 * ((1 - obstacleratio) * (gridsize * gridsize))) {
                            continue;
                        }

                        int i = 0;
                        int seed = random.nextInt(10000);
                        do {

                            try {
                                ToyWorldMultiAgentProblemInstance problemInstance = null;
                                if (scenario.equals("random")) {
                                    problemInstance = new ToyWorldRandomProblemInstance(
                                            gridsize, gridsize, 3 * gridsize,
                                            nagents, separation / 2,
                                            obstacleratio, obstaclesize, seed);
                                }

                                if (scenario.equals("superconflict")) {
                                    problemInstance = new ToyWorldSuperconflictProblemInstance(
                                            gridsize, gridsize, 3 * gridsize,
                                            nagents, separation / 2,
                                            obstacleratio, obstaclesize, seed);
                                }
                                if (pathsExist(problemInstance)) {
                                    for (String alg : algorithms) {
                                        String s = instanceNo + " " + alg + " "
                                                + scenario + " " + gridsize
                                                + " " + gridsize + " "
                                                + nagents + " "
                                                + formatDouble(separation)
                                                + " "
                                                + formatDouble(obstacleratio)
                                                + " " + obstaclesize + " "
                                                + seed + "\n";
                                        System.out.print(s);
                                    }
                                    instanceNo++;
                                    i++;
                                    seed++;
                                } else {
                                    // graph is not connected
                                    seed++;
                                }
                            } catch (CannotPlaceObstaclesException e) {
                                seed++;
                                System.out.println("Cannot place obstalces");
                            } catch (CannotPlaceAgentsException e) {
                                seed++;
                                System.out.println("Cannot place agents");
                            }

                        } while (i < n);
                    }
                }
            }
        }

        return sb.toString();
    }

    private static String formatDouble(double d) {
        return String.format("%.2f", d);
    }

    private static boolean pathsExist(ToyWorldMultiAgentProblemInstance problemInstance) {
        DirectedGraph<Waypoint, SpatialManeuver> graph = problemInstance.getGraph();
        SpatialPoint[] starts = problemInstance.getStarts();
        SpatialPoint[] targets = problemInstance.getTargets();
        ConnectivityInspector<Waypoint, SpatialManeuver> inspector = new ConnectivityInspector<Waypoint, SpatialManeuver>(graph);

        for (int i=0; i<starts.length;i++) {
            boolean pathExists = inspector.pathExists(
                    SpatialGraphs.getNearestVertex(graph, starts[i]),
                    SpatialGraphs.getNearestVertex(graph, targets[i]));
            if (!pathExists) {
                return false;
            }
        }

        return true;
    }

    public static void main(String[] args) {
        System.out.println(generate());
    }
}
