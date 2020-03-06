package trclib;

import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;

public class PathFollowingTest extends PathFollowingTestBase
{
    private TrcPidDrive pidDrive;
    private TrcHolonomicPurePursuitDrive purePursuit;
    private TrcPidController xPid, yPid, turnPid;

    @Before
    public void setup()
    {
        super.setup();

        xPid = new TrcPidController("xPid", new TrcPidController.PidCoefficients(8, 0, 2), 0.6,
            driveBase::getXPosition);
        yPid = new TrcPidController("yPid", new TrcPidController.PidCoefficients(8, 0, 2), 0.6,
            driveBase::getYPosition);
        turnPid = new TrcPidController("turnPid", new TrcPidController.PidCoefficients(0.01, 0, 0.01), 3,
            driveBase::getHeading);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, xPid, yPid, turnPid);

        purePursuit = new TrcHolonomicPurePursuitDrive("", driveBase, 12, tolerance, turnTolerance,
            new TrcPidController.PidCoefficients(8), new TrcPidController.PidCoefficients(0.01, 0, 0.01),
            new TrcPidController.PidCoefficients(0.01, 0, 0, 1 / 100.0));
        purePursuit.setMoveOutputLimit(0.6);
    }

    /*
    @Test
    public void pidDriveTest()
    {
        TrcPose2D[][] poses = new TrcPose2D[][] { { new TrcPose2D(0, 0), null }, { new TrcPose2D(60, 60), null },
            { new TrcPose2D(0, 60), null }, { new TrcPose2D(0, 0), null } };
        TrcPath path = new TrcPath(
            Arrays.stream(poses).map(p -> new TrcWaypoint(p[0], p[1])).toArray(TrcWaypoint[]::new));
        s.addPath(path);
        s.start();

        xPid.setNoOscillation(true);
        yPid.setNoOscillation(true);

        TrcEvent event = new TrcEvent("");
        pidDrive.setRelativeTarget(60, 60, 0, event);
        assertTimeout(event, 4000);
        pidDrive.setRelativeTarget(-60, 0, 0, event);
        assertTimeout(event, 4000);
        xPid.setNoOscillation(false);
        yPid.setNoOscillation(false);
        pidDrive.setRelativeTarget(0, -60, 0, event);
        System.out.println(pidDrive.getAbsoluteTargetPose());
        assertTimeout(event, 4000);
        assertPosition(new TrcPose2D(), new TrcPose2D());
    }
     */

    @Test
    public void purePursuitTest()
    {
        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(40, 40, 90),
            new TrcPose2D(40, 100, 90) };
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));
        path = path.trapezoidVelocity(100, 200);
        s.addPath(path);
        s.start();

        TrcEvent event = new TrcEvent("");
        TrcPose2D ref = driveBase.getFieldPosition();
        purePursuit.start(path, event, 0);

        assertTimeout(event, 5000);

        TrcPose2D end = poses[poses.length - 1];
        assertPosition(driveBase.getFieldPosition().relativeTo(ref), end);
    }

    @Test
    public void purePursuitV2Test()
    {
        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(40, 40, 90),
            new TrcPose2D(40, 100, 90) };
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));
        path = path.trapezoidVelocity(driveBase.maxVel, 0.7 * driveBase.maxAccel);
        //        path.getWaypoint(0).velocity = 20;
        path.getWaypoint(path.getSize() - 2).acceleration = 0;
        path.inferTimeSteps();
        s.addPath(path);
        s.start();

        TrcEvent event = new TrcEvent("");
        TrcPose2D ref = driveBase.getFieldPosition();

        TrcHolonomicPurePursuitDriveV2 purePursuit = new TrcHolonomicPurePursuitDriveV2("", driveBase, 12, tolerance,
            turnTolerance, 1, new TrcPidController.PidCoefficients(0.03, 0, 0.01),
            new TrcPidController.PidCoefficients(0, 5, 0, 1 / driveBase.maxVel), 0.7 / driveBase.maxAccel);
        //        purePursuit.setMoveOutputLimit(0.6);
        purePursuit.start(path, event, 0);

        assertTimeout(event, 5000);

        TrcPose2D end = poses[poses.length - 1];
        assertPosition(driveBase.getFieldPosition().relativeTo(ref), end);
    }
}
