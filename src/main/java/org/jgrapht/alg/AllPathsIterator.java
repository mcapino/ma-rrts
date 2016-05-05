package org.jgrapht.alg;

import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.Graphs;
import org.jgrapht.alg.specifics.*;

import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.planner.PlannedPathImpl;

public class AllPathsIterator<V, E> implements Iterator<PlannedPath<V, E>>{

    /*
     * Key is a visited edge (part of the path)
     * Value is a queue of outgoing edges to be used later
     */
    Map<V, Queue<E>> seen = new HashMap<V, Queue<E>>();

    private Specifics<V, E> specifics;

    private final Graph<V, E> graph;
    private final V startVertex;
    private final V endVertex;
    
    private Deque<E> currentPath;
    private Deque<V> currentPathV;

    private boolean isCurrentPathUsed;

    public AllPathsIterator(Graph<V, E> graph, V startVertex, V endVertex) {
        this.graph = graph;
        this.startVertex = startVertex;
        this.endVertex = endVertex;
        
        specifics = createGraphSpecifics(graph);
        
        currentPath = new LinkedList<E>();
        currentPathV = new LinkedList<V>();
        findFirstPath(startVertex);
        isCurrentPathUsed = false;
    }
    
    private boolean findFirstPath(V vertex) {
        if (seen.containsKey(vertex)) {
            return false;
        }

        currentPathV.add(vertex);
        
        if (vertex.equals(endVertex)) {
            return true;
        }
        
        Queue<E> edges = new LinkedList<E>( specifics.edgesOf(vertex) );
        seen.put(vertex, edges);
        
        while (!edges.isEmpty()) {
            E edge = edges.poll();
            
            currentPath.add(edge);
            
            V oppositeV = Graphs.getOppositeVertex(graph, edge, vertex);
            
            if ( findFirstPath(oppositeV) ) {
                return true;
            } else {
                currentPath.removeLast();
            }
        }

        currentPathV.removeLast();
        seen.remove(vertex);

        return false;
    }

    private void findNextPath() {
        while (!currentPath.isEmpty()) {
            V vertex = currentPathV.removeLast();
            currentPath.removeLast();
            seen.remove(vertex);

            V lastVertex = currentPathV.getLast();
            
            Queue<E> edges = seen.get(lastVertex);

            while (!edges.isEmpty()) {
                E edge = edges.poll();
                
                currentPath.add(edge);
                
                V oppositeV = Graphs.getOppositeVertex(graph, edge, lastVertex);
                
                if ( findFirstPath(oppositeV) ) {
                    return;
                } else {
                    currentPath.removeLast();
                }
            }
        }
        currentPath = null;
    }

    @Override
    public boolean hasNext() {
        if (isCurrentPathUsed) {
            findNextPath();
            isCurrentPathUsed = false;
        }
        return currentPath != null;
    }

    @Override
    public PlannedPath<V, E> next() {
        if (!hasNext()) {
            throw new NoSuchElementException();
        }
        isCurrentPathUsed = true;
        return new PlannedPathImpl<V, E>(graph, startVertex, endVertex, new ArrayList<E>(currentPath), 0.0);
    }

    @Override
    public void remove() {
        throw new UnsupportedOperationException(); 
    }

    static <V, E> Specifics<V, E> createGraphSpecifics(Graph<V, E> g)
    {
        if (g instanceof DirectedGraph<?, ?>) {
            return new DirectedSpecifics<V, E>((DirectedGraph<V, E>) g);
        } else {
            return new UndirectedSpecifics<V, E>(g);
        }
    }
}
