package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.Random;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.DistanceFunction;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class BiasedManeuverGraphDomain<S extends SpatialPoint, E extends SpatialManeuver> extends ManeuverGraphDomain<S,E>{

    double tryTargetRatio;
    public BiasedManeuverGraphDomain(Graph<S, E> graph, S target,
            DistanceFunction<S> distance, double maxExtensionCost, double tryTargetRatio, Random random) {
        super(graph, target, distance, maxExtensionCost, random);
        this.tryTargetRatio = tryTargetRatio;
    }

    @Override
    public S sampleState() {
        S sample;

        if (random.nextDouble() < tryTargetRatio) {
            sample = target;
        }
        else {
            sample = super.sampleState();
        }

        return sample;
    }
}
