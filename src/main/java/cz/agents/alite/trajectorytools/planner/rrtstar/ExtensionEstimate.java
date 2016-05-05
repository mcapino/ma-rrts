package cz.agents.alite.trajectorytools.planner.rrtstar;

public class ExtensionEstimate {
    final public double cost;
    final public boolean exact;

    public ExtensionEstimate(double cost, boolean exact) {
        super();
        this.cost = cost;
        this.exact = exact;
    }
}