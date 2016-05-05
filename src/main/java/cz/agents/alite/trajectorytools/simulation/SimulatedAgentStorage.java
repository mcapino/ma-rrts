package cz.agents.alite.trajectorytools.simulation;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class SimulatedAgentStorage {

    private Map<String, OrientedPoint> agents = new LinkedHashMap<String, OrientedPoint>();

    public Map<String, OrientedPoint> getAgents() {
        return Collections.unmodifiableMap(agents);
    }

    synchronized public void set(String name, OrientedPoint position) {
        agents.put(name, position);
    }
}
