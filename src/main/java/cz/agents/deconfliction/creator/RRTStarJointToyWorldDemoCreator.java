package cz.agents.deconfliction.creator;

import java.awt.Color;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;

import org.jgrapht.GraphPath;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalManeuver;
import cz.agents.alite.trajectorytools.graph.jointspatiotemporal.JointSpatioTemporalState;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.rrtstar.GoalBiasedJointToyWorldDomain;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.JointToyWorldRRTStarLayer;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer;
import cz.agents.alite.trajectorytools.graph.jointtoyworld.vis.MissionsLayer.MissionsProvider;
import cz.agents.alite.trajectorytools.graph.spatial.region.SpaceRegion;
import cz.agents.alite.trajectorytools.graph.spatiotemporal.region.SpaceTimeRegion;
import cz.agents.alite.trajectorytools.planner.rrtstar.Domain;
import cz.agents.alite.trajectorytools.planner.rrtstar.RRTStarPlanner;
import cz.agents.alite.trajectorytools.simulation.SimulatedAgentEnvironment;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.vis.RegionsLayer;
import cz.agents.alite.trajectorytools.vis.RegionsLayer.RegionsProvider;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer.TimeProvider;
import cz.agents.alite.trajectorytools.vis.SimulationControlLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer.TrajectoriesProvider;
import cz.agents.alite.trajectorytools.vis.projection.DefaultProjection;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.deconfliction.probleminstance.ToyWorldMultiAgentProblemInstance;
import cz.agents.deconfliction.probleminstance.ToyWorldRandomProblemInstance;
import cz.agents.deconfliction.util.JointManeuversToTrajectoriesConverter;


public class RRTStarJointToyWorldDemoCreator implements Creator {

    RRTStarPlanner<JointSpatioTemporalState, JointSpatioTemporalManeuver> rrtstar;
    double gamma = 900;

    private double SPEED = 1.0;

    private ToyWorldMultiAgentProblemInstance problem;
    private Trajectory[] trajectories;
    private TimePoint[] starts;

    Trajectory[] nominalTrajectories;
    SimulatedAgentEnvironment simulation = new SimulatedAgentEnvironment();

    @Override
    public void init(String[] args) {

    }

    @Override
    public void create() {

        /*Mission[] missions = AgentMissionGenerator.generateSuperconflict(2,
                new SpatialPoint(500, 500, 0),
                400);*/

        problem = new ToyWorldRandomProblemInstance(2, 8, 100, 3, 0.8, 0.00, 12);

        trajectories = new Trajectory[problem.nAgents()];
        nominalTrajectories = new Trajectory[problem.nAgents()];

        starts = new TimePoint[problem.nAgents()];
        for (int i=0; i<problem.nAgents(); i++) {
            starts[i] = new TimePoint(problem.getStart(i), 0);
        }

        Domain<JointSpatioTemporalState, JointSpatioTemporalManeuver> domain = new GoalBiasedJointToyWorldDomain(problem, SPEED, new Random(1));

        rrtstar = new RRTStarPlanner<JointSpatioTemporalState, JointSpatioTemporalManeuver>(domain, new JointSpatioTemporalState(starts), gamma);
        createVisualization();

        double bestCost = Double.POSITIVE_INFINITY;
        int n=30000;
        for (int i=0; i<n; i++) {
            rrtstar.iterate();

            if (rrtstar.getBestVertex() != null && rrtstar.getBestVertex().getCostFromRoot() < bestCost) {
                bestCost = rrtstar.getBestVertex().getCostFromRoot();
                System.out.println("Iteration: " + i + " Best path cost: " + bestCost);
                GraphPath<JointSpatioTemporalState, JointSpatioTemporalManeuver> path = rrtstar.getBestPath();

                trajectories = JointManeuversToTrajectoriesConverter.convert(path);

                for (int j=0; j<trajectories.length; j++) {
                    simulation.updateTrajectory("t"+j, trajectories[j]);
                }
            }


//			try {
//				Thread.sleep(1);
//			} catch (InterruptedException e) {
//				e.printStackTrace();
//			}
        }

    }

    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768);
        VisManager.init();
        VisManager.setSceneParam(new SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(500, 500);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.35;
            }

        });

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));



        // X-Y TOP View ////////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, point.y);
            }
        });

        // Missions shown only in X-Y
        VisManager.registerLayer(MissionsLayer.create(new MissionsProvider(){

            @Override
            public SpatialPoint[] getStarts() {
                return problem.getStarts();
            }

            @Override
            public SpatialPoint[] getTargets() {
                return problem.getTargets();
            } },
            new DefaultProjection<SpatialPoint>(), Color.RED));

        // Y-T View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(-point.w - 50, point.y);
            }
        });


        // X-T View ///////////////////////////////////////////////////////////////

        createView( new ProjectionTo2d<TimePoint>() {

            @Override
            public Point2d project(TimePoint point) {
                return new Point2d(point.x, -point.w - 50);
            }
        });

        ///////////////////////////////////////////////////////////////////////////

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());

        // Simulation Control
        VisManager.registerLayer(SimulationControlLayer.create(simulation));
    }

    private void createView(ProjectionTo2d<TimePoint> projection) {
        // graph
        VisManager.registerLayer(JointToyWorldRRTStarLayer.create(rrtstar, projection, SPEED, 1, 4));

        VisManager.registerLayer(TrajectoriesLayer.create(new TrajectoriesProvider() {

            @Override
            public Trajectory[] getTrajectories() {
                List<Trajectory> trajs = new LinkedList<Trajectory>(Arrays.asList(trajectories));
                //trajs.addAll(Arrays.asList(decoupledTrajectories));
                return trajs.toArray(new Trajectory[1]);
            }
        }, projection, 0.5, problem.getBounds4d().getCorner2().w, 't'));

        VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {

            @Override
            public Collection<SpaceTimeRegion> getSpaceTimeRegions() {
                LinkedList<SpaceTimeRegion> regions = new LinkedList<SpaceTimeRegion>();
                regions.add(problem.getBounds4d());
                return regions;
            }

            @Override
            public Collection<SpaceRegion> getSpaceRegions() {
                return problem.getObstacles();
            }
        },
        projection,
        Color.BLACK, 1));

        VisManager.registerLayer(SimulatedCylindricAgentLayer.create(simulation.getAgentStorage(), projection, new TimeProvider() {

            @Override
            public double getTime() {
                return simulation.getTime();
            }
        }, problem.getAgentSizeRadius(), 5));
    }



  }
