package cz.agents.alite.trajectorytools.graph.spatial.rrtstar;

import java.util.Random;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.DistanceFunction;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class ExploringBiasedManeuverGraphDomain<S extends SpatialPoint, E extends SpatialManeuver> extends ManeuverGraphDomain<S,E>{

    double tryTargetRatio;
    boolean explore;
    public ExploringBiasedManeuverGraphDomain(Graph<S, E> graph, S target,
            DistanceFunction<S> distance, double maxExtensionCost, double tryTargetRatio, Random random) {
        super(graph, target, distance, maxExtensionCost, random);
        this.tryTargetRatio = tryTargetRatio;
        this.explore = true;
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

    @Override
    public double estimateCostToGo(S s) {
    	if (explore) {
    		return Double.NEGATIVE_INFINITY;
    	} else {
    		return super.estimateCostToGo(s);
    	}
    }

    public void setExploreMode(boolean explore) {
    	this.explore = explore;
    }



}
