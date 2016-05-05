package org.jgrapht;

public class DummyEdgeFactory<V,E> implements EdgeFactory<V, E> {

    @Override
    public E createEdge(V sourceVertex, V targetVertex) {
        throw new RuntimeException("This is a \"dummy\" edge factory not able to create edge. It should have never been invoked.");
    }

}
