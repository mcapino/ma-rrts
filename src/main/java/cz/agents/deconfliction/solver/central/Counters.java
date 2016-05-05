package cz.agents.deconfliction.solver.central;

public class Counters {
    static public long statesExpanded = 0;
    static public long iterations = 0;
    
    static public long createSpecificsNanos = 0;
    static public long edgesOfNanos = 0;
    static public long oppositeVertexNanos = 0;
    static public long greedySearchNanos = 0;
    static public long greedySearchNum = 0;
    static public long collisionCheckNanos = 0;
    static public long collisionCheckNum = 0;
    static public long collisionCheckApproxNum = 0;
}
