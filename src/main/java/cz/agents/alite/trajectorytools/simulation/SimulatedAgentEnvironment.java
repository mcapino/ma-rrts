package cz.agents.alite.trajectorytools.simulation;

import java.util.HashMap;
import java.util.Map;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class SimulatedAgentEnvironment {

    private SimulatedAgentStorage agentStorage;
    private Map<String, Trajectory> trajectories = new HashMap<String, Trajectory>();
    private Simulation simulation;
    //private defaultSpeed = 1.0;

    public SimulatedAgentEnvironment() {
        this.simulation = new Simulation();
        this.simulation.setSimulationSpeed(0.1);
        agentStorage = new SimulatedAgentStorage();
    }

    public SimulatedAgentStorage getAgentStorage() {
        return agentStorage;
    }
    
    public void clearTrajectories() {
    	trajectories = new HashMap<String, Trajectory>();
    }

    public void updateTrajectory(String name, Trajectory trajectory) {
        trajectories.put(name, trajectory);
    };

    public void updateTrajectories(Map<String, Trajectory> trajectories) {
        this.trajectories = new HashMap<String, Trajectory>(trajectories);
        for (String agentName: trajectories.keySet()) {
            if (trajectories.get(agentName) != null) {
                OrientedPoint startPos = trajectories.get(agentName).getPosition(trajectories.get(agentName).getMinTime());
                if (startPos != null) {
                    if (!agentStorage.getAgents().containsKey(agentName)) {
                        getAgentStorage().set(agentName, startPos);
                    }
                }
            }
        }
    }

    private void simulateFlight(final String name, final Trajectory trajectory, double samplingInterval) {
        final SimulatedAgentStorage storage = agentStorage;
        for (double t=trajectory.getMinTime(); t <= trajectory.getMaxTime(); t += samplingInterval) {
            final double time = t;
                simulation.addEvent(new EventHandler() {

                    @Override
                    public void handleEvent(Event event) {
                        storage.set(name, trajectory.getPosition(time));
                    }

                    @Override
                    public EventProcessor getEventProcessor() {
                        return getEventProcessor();
                    }
                }, (long) (time*1000+1));
        }
    }

    public void startFlightSimulation(double samplingInterval, final double simulationSpeed) {
        simulation.clearQueue();
        simulation = new Simulation();
        simulation.clearQueue();

        for(String agentName : trajectories.keySet()) {
            simulateFlight(agentName, trajectories.get(agentName), samplingInterval);
        }

        new Thread() {

            @Override
            public void run() {
                simulation.setSimulationSpeed(simulationSpeed);
                simulation.run();
            }

        }.start();

    }

    public double getTime() {
        return simulation.getCurrentTime() / 1000.0;
    }

    public Simulation getSimulation() {
        return simulation;
    }

}
