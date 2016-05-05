package org.jgrapht;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

public class SingleVertexGraphPath<V,E> implements GraphPath<V, E> {
    Graph<V, E> graph;
    private V start;

    public SingleVertexGraphPath(Graph<V, E> graph, V start) {
        super();
        this.graph = graph;
        this.start = start;
    }

    @Override
    public Graph<V, E> getGraph() {
        return graph;
    }

    @Override
    public V getStartVertex() {
        return start;
    }

    @Override
    public V getEndVertex() {
        return start;
    }

    @Override
    public List<E> getEdgeList() {
        List<E> list = new LinkedList<E>();
        return list;
    }

    @Override
    public double getWeight() {
        return 0;
    }

}