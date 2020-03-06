package trclib;

import org.junit.After;
import org.junit.Before;
import team492.RobotInfo;
import trclib.simulator.SimulatedHolonomicDrivebase;
import trclib.simulator.Simulator;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public abstract class PathFollowingTestBase
{
    protected SimulatedHolonomicDrivebase driveBase;
    protected Simulator s;
    protected double tolerance, turnTolerance;

    @Before
    public void setup()
    {
        driveBase = new SimulatedHolonomicDrivebase(100, 300, 6, 10);
        driveBase.setOdometryScales(1, 1, 1.0 / 15);
        driveBase.setOdometryEnabled(true);
        driveBase.setFieldPosition(new TrcPose2D());
        s = new Simulator(RobotInfo.FIELD_WIDTH, RobotInfo.FIELD_LENGTH, 2, 30, driveBase);
        tolerance = 2;
        turnTolerance = 3;
        TrcUtil.recordModeStartTime();
    }

    @After
    public void tearDown()
    {
        s.stop();
    }

    protected void assertTimeout(TrcEvent event, long millis)
    {
        AtomicBoolean done = new AtomicBoolean(false);
        Thread testThread = Thread.currentThread();
        Thread t = new Thread(() -> {
            if (s.blockForEvent(event))
            {
                done.set(true);
                testThread.interrupt();
            }
        });
        t.start();
        try
        {
            Thread.sleep(millis);
            t.interrupt();
            if (!done.get())
            {
                fail("Timed out!");
            }
        }
        catch (InterruptedException ignored)
        {
        }
    }

    protected void assertPosition(TrcPose2D curr, TrcPose2D position)
    {
        assertEquals(position.x, curr.x, tolerance);
        assertEquals(position.y, curr.y, tolerance);
        assertEquals(position.angle, curr.angle, turnTolerance);
    }
}
