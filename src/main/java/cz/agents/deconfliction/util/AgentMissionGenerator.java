package cz.agents.deconfliction.util;

import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class AgentMissionGenerator {

    public static class Mission {
        public SpatialPoint start;
        public SpatialPoint end;

        public Mission(SpatialPoint start, SpatialPoint end) {
            super();
            this.start = start;
            this.end = end;
        }

        @Override
        public String toString() {
            return "Mission [start=" + start + ", end=" + end + "]";
        }


    }

    public static Mission[] generateSuperconflict(int nAgents, SpatialPoint center, double circleRadius) {
        Mission[] startEnds = new Mission[nAgents];

        for (int i = 0; i < nAgents; i++ ) {

            double angle = i*(2*Math.PI/(double)nAgents);
            SpatialPoint startPoint = new SpatialPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius,center.z);
            SpatialPoint endPoint = new SpatialPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius,center.z);

            startEnds[i] = new Mission(startPoint, endPoint);
        }
        return startEnds;

    }

    /*

    public static Mission[] generateSuperconflicts(ManeuverGraph maneuvers, int[] sizes, Point[] centers, int[] radiuses) {

        ArrayList<Mission> resultList = new ArrayList<Mission>();

        for (int i = 0; i < sizes.length; i++) {
            Mission[] superconflict = generateSuperconflict(maneuvers, sizes[i], centers[i], radiuses[i]);
            resultList.addAll(Arrays.asList(superconflict));
        }

        return (Mission[]) resultList.toArray(new Mission[resultList.size()]);
    }


    public static Mission[] generateTest(ManeuverGraph maneuvers) {
        Mission[] missions = new Mission[3];

        missions[0] = new Mission(maneuvers.getNearestWaypoint(new Point(2,0,0)), maneuvers.getNearestWaypoint(new Point(3,10,0)));
        missions[1] = new Mission(maneuvers.getNearestWaypoint(new Point(5,5,0)), maneuvers.getNearestWaypoint(new Point(10,5,0)));
        missions[2] = new Mission(maneuvers.getNearestWaypoint(new Point(7.5,7.5,0)), maneuvers.getNearestWaypoint(new Point(7.0,0,0)));

        return missions;
    }

    public static Mission[] generateRandom(Random random, ManeuverGraph maneuvers, int nAgents, double maxx, double maxy, double separation) {
        Mission[] missions = new Mission[nAgents];
        Set<Waypoint> startWaypoints = new HashSet<Waypoint>();
        Set<Waypoint> endWaypoints = new HashSet<Waypoint>();

        for (int i = 0; i < nAgents; i++) {
            Waypoint startWp;
            Waypoint endWp;

            Point start;
            Point end;
            do {
                start = new Point(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                startWp = maneuvers.getNearestWaypoint(start);

                end = new Point(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                endWp = maneuvers.getNearestWaypoint(end);
            }
            while (!separated(startWp, startWaypoints, 1.5*separation) || !separated(endWp, endWaypoints, 1.5*separation) || startWp.equals(endWp));
            startWaypoints.add(startWp);
            endWaypoints.add(endWp);

            missions[i] = new Mission(startWp, endWp);
        }
        return missions;
    }

    public static Mission[] generateRandomSimilarLength(Random random, ManeuverGraph maneuvers, int nAgents, double maxx, double maxy, double radius, double separation) {
        Mission[] missions = new Mission[nAgents];
        Set<Waypoint> startWaypoints = new HashSet<Waypoint>();
        Set<Waypoint> endWaypoints = new HashSet<Waypoint>();

        for (int i = 0; i < nAgents; i++ ) {
            Waypoint startWp;
            Waypoint endWp;

            Point start;
            do {
                double r = radius * (0.5 + random.nextDouble());
                start = new Point(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                startWp = maneuvers.getNearestWaypoint(start);

                double angle = (2*Math.PI*random.nextDouble());

                Point end = new Point(start.x + Math.cos(angle)*r, start.y + Math.sin(angle)*r,0);
                endWp = maneuvers.getNearestWaypoint(end);
            }
            while (!separated(startWp, startWaypoints, 1.5*separation) || !separated(endWp, endWaypoints, 1.5*separation) || startWp.equals(endWp));
            startWaypoints.add(startWp);
            endWaypoints.add(endWp);

            missions[i] = new Mission(startWp, endWp);
        }
        return missions;
    }

    public static Mission[] generateSuperconflict(ManeuverGraph maneuvers, int nAgents, Point center, double circleRadius) {
        Mission[] startEnds = new Mission[nAgents];

        for (int i = 0; i < nAgents; i++ ) {

            double angle = i*(2*Math.PI/(double)nAgents);
            OrientedPoint startPoint = new OrientedPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius,0,0,1,0);
            OrientedPoint endPoint = new OrientedPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius,0,0,1,0);

            Waypoint start = maneuvers.getNearestWaypoint(startPoint);
            Waypoint end = maneuvers.getNearestWaypoint(endPoint);

            startEnds[i] = new Mission(start, end);
        }
        return startEnds;

    }

    public static Mission[] generateSpiralSuperconflict(ManeuverGraph maneuvers, int nAgents, Point center, double minRadius, double maxRadius, double angleOffset) {
        Mission[] startEnds = new Mission[nAgents];

        double radiusStep = (maxRadius - minRadius) / (nAgents-1);

        for (int i = 0; i < nAgents; i++ ) {

            double circleRadius = minRadius + i * radiusStep;
            double angle = i*(2*Math.PI/(double)nAgents) + angleOffset;
            OrientedPoint startPoint = new OrientedPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius,0,0,1,0);
            OrientedPoint endPoint = new OrientedPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius,0,0,1,0);

            Waypoint start = maneuvers.getNearestWaypoint(startPoint);
            Waypoint end = maneuvers.getNearestWaypoint(endPoint);

            startEnds[i] = new Mission(start, end);
        }
        return startEnds;

    }

    private static boolean separated(Waypoint candidate, Set<Waypoint> usedWaypoints, double separation) {
        for (Waypoint wp : usedWaypoints) {
            if (wp.distance(candidate) <= separation) {
                return false;
            }
        }
        return true;
    }*/
}
