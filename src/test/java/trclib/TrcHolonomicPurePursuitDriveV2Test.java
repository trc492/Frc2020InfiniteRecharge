package trclib;

import org.junit.Before;
import org.junit.Test;
import trclib.simulator.SimulatedHolonomicDrivebase;

import java.util.Arrays;

import static org.junit.Assert.*;

public class TrcHolonomicPurePursuitDriveV2Test
{
    private TrcHolonomicPurePursuitDriveV2 purePursuit;

    @Before
    public void setup()
    {
        SimulatedHolonomicDrivebase driveBase = new SimulatedHolonomicDrivebase(60, 120, 700, 1000);
        purePursuit = new TrcHolonomicPurePursuitDriveV2("", driveBase, 12, 6, 1,
            new TrcPidController.PidCoefficients(1), 1.0/120);
    }

    @Test
    public void getFollowingPointTest()
    {
        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(0, 40), new TrcPose2D(40, 40) };
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));
        path = path.trapezoidVelocity(100, 200);
        purePursuit.start(path);
        purePursuit.setFollowingDistance(12);
        assertEquals(new TrcPose2D(0, 12), purePursuit.getFollowingPoint(new TrcPose2D(0, 0)).getPositionPose());
        assertEquals(new TrcPose2D(0, 24), purePursuit.getFollowingPoint(new TrcPose2D(0, 12)).getPositionPose());
        purePursuit.setPositionToleranceAndFollowingDistance(1, 5);
        assertEquals(new TrcPose2D(0, 12), purePursuit.getFollowingPoint(new TrcPose2D(-3, 8)).getPositionPose());
        assertEquals(new TrcPose2D(0, 35), purePursuit.getFollowingPoint(new TrcPose2D(5, 35)).getPositionPose());
        purePursuit.setFollowingDistance(5 * Math.sqrt(2));
        assertEquals(new TrcPose2D(0, 40), purePursuit.getFollowingPoint(new TrcPose2D(5, 35)).getPositionPose());
    }

    @Test
    public void getClosestPointTest()
    {
        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(0, 40), new TrcPose2D(40, 40) };
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));
        path = path.trapezoidVelocity(100, 200);
        purePursuit.start(path);
        assertEquals(new TrcPose2D(0, 0), purePursuit.getTargetPointDistParameterized(new TrcPose2D()).getPositionPose());
        assertEquals(new TrcPose2D(0, 20), purePursuit.getTargetPointDistParameterized(new TrcPose2D(5, 20)).getPositionPose());
        assertEquals(new TrcPose2D(0, 35), purePursuit.getTargetPointDistParameterized(new TrcPose2D(5, 35)).getPositionPose());
        assertEquals(new TrcPose2D(0, 0), purePursuit.getTargetPointDistParameterized(new TrcPose2D(-5, -10)).getPositionPose());
        assertEquals(new TrcPose2D(5, 40), purePursuit.getTargetPointDistParameterized(new TrcPose2D(5, 38)).getPositionPose());
        assertEquals(new TrcPose2D(40, 40), purePursuit.getTargetPointDistParameterized(new TrcPose2D(50, 38)).getPositionPose());
    }
}