package cz.agents.alite.trajectorytools.util;

/**
 * A waypoint is a {@link SpatialPoint} having an identifier.
 */
public class Waypoint extends SpatialPoint implements Comparable<Waypoint> {
    private static final long serialVersionUID = -9146741978852925425L;
    public static int globalIdCounter = 0;

    public final int id;

    public Waypoint(double x, double y) {
        this(globalIdCounter, x, y);
    }

    public Waypoint(int order, double x, double y) {
        super(x,y,0.0);
        this.id = order;
        if (order >= globalIdCounter) {
            globalIdCounter = order + 1;
        }
    }

    @Override
    public int compareTo(Waypoint other) {
        return id - other.id;
    }

    @Override
    public String toString() {
        return "#" + id;
        //return "#" + order + " "+ super.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof Waypoint) {
            Waypoint other = (Waypoint) o;
            return id == other.id;
        } else {
            return false;
        }
    }
}
