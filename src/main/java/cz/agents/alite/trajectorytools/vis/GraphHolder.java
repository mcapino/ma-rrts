package cz.agents.alite.trajectorytools.vis;

import org.jgrapht.Graph;

public class GraphHolder<V, E> {
	public Graph<V,E> graph = null;

	public GraphHolder () {
	}

	public GraphHolder (Graph<V, E> graph) {
        this.graph = graph;
	}
}