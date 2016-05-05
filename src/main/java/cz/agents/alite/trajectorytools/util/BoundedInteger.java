package cz.agents.alite.trajectorytools.util;

public class BoundedInteger extends Number {

    private static final long serialVersionUID = 9196227379128365431L;

    private final int bound;
    private final int value;

    public BoundedInteger(int value, int bound) {
        this.bound = bound;
        this.value = normalize(value);
    }


    public BoundedInteger plus(Number that) {
        return new BoundedInteger(value + that.intValue(), bound);
    }

    @Override
    public double doubleValue() {
        return value;
    }


    @Override
    public float floatValue() {
        return value;
    }


    @Override
    public int intValue() {
        return value;
    }


    @Override
    public long longValue() {
        return value;
    }

    private int normalize(int v) {
        if (v < 0) {
            return bound + (v % bound);
        } else {
            return v % bound;
        }
    }

}
