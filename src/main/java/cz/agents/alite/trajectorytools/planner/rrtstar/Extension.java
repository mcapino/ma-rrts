package cz.agents.alite.trajectorytools.planner.rrtstar;

public class Extension<S,E> {
    final public S source;
    final public S target;
    final public E edge;
    final public double cost;
    final public boolean exact;

    public Extension(S source, S target, E edge, double cost, boolean exact) {
        super();
        this.source = source;
        this.target = target;
        this.edge = edge;
        this.cost = cost;
        this.exact = exact;
    }
}