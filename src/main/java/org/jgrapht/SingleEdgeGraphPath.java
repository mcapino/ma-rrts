package org.jgrapht;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;

public class SingleEdgeGraphPath<V,E> implements GraphPath<V, E> {
    Graph<V, E> graph;
    private V start;
    private V end;
    private E edge;
    private double weight;

    public SingleEdgeGraphPath(Graph<V, E> graph, V start, E edge, V end, double weight) {
        super();
        this.graph = graph;
        this.edge = edge;
        this.start = start;
        this.end = end;
        this.weight = weight;
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
        return end;
    }

    @Override
    public List<E> getEdgeList() {
        List<E> list = new LinkedList<E>();
        list.add(edge);
        return list;
    }

    @Override
    public double getWeight() {
        return weight;
    }

}
