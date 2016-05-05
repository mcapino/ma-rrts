package cz.agents.alite.trajectorytools.graph.spatial;

import java.util.Set;

import org.jgrapht.Graph;

public interface GraphWithObstacles<V, E> extends Graph<V, E> {

    public Set<V> getObstacles();

    public void addObstacle(V obstacle);

    void refresh();
}
