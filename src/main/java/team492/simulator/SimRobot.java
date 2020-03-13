package team492.simulator;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frclib.FrcPath;
import frclib.FrcSwervePathFollower;
import team492.RobotInfo;
import trclib.TrcPath;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcUtil;
import trclib.TrcWaypoint;

import java.util.Arrays;
import java.util.stream.IntStream;

public class SimRobot extends TimedRobot
{
    private Simulator s;
    private SimulatedSwerveDrive driveBase;
    private FrcSwervePathFollower follower;

    @Override
    public void robotInit()
    {
        driveBase = new SimulatedSwerveDrive(200, 1000);
        s = new Simulator(RobotInfo.FIELD_WIDTH, RobotInfo.FIELD_LENGTH, 2, 30, driveBase);
        s.start();
        TrcUtil.recordModeStartTime();
    }

    @Override
    public void teleopInit()
    {
        driveBase.setOdometryEnabled(true);
    }

    @Override
    public void teleopPeriodic()
    {
        driveBase.holonomicDrive(null, 0, 0.5, 0.3, false, driveBase.getHeading());
    }

    @Override
    public void autonomousInit()
    {
        TrcUtil.recordModeStartTime();

        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(0, 0), new TrcPose2D(40, 40), new TrcPose2D(40, 100, -90) };
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));
        TrajectoryConfig config = FrcPath.createSwerveConfig(driveBase, 0.8 * driveBase.maxVel, 300, 1000);
        Trajectory trajectory = FrcPath.createHolonomicTrajectory(path, config);
        double t = trajectory.getTotalTimeSeconds();

        int numPoints = (int) Math.ceil(t / 0.05);
        TrcPose2D[] traj = IntStream.rangeClosed(0, numPoints).mapToDouble(i -> i * t / numPoints)
            .mapToObj(trajectory::sample).map(s -> s.poseMeters).map(Pose2d::getTranslation)
            .map(e -> new TrcPose2D(-e.getY() / TrcUtil.METERS_PER_INCH, e.getX() / TrcUtil.METERS_PER_INCH))
            .toArray(TrcPose2D[]::new);
        TrcPath trajPath = new TrcPath(
            Arrays.stream(traj).map(p -> new TrcWaypoint(p, null)).toArray(TrcWaypoint[]::new));

        TrcPidController.PidCoefficients movePid = new TrcPidController.PidCoefficients(0.08, 0, 0);
        TrcPidController.PidCoefficients turnPid = new TrcPidController.PidCoefficients(5, 0, 0.0);
        follower = new FrcSwervePathFollower("blah", driveBase, movePid, turnPid, driveBase.maxRotVel, 1000,
            driveBase.maxVel);

        driveBase.setFieldPosition(new TrcPose2D());
        driveBase.setOdometryEnabled(true);
        follower.start(trajectory);
        s.addPath(path);
        s.addPath(trajPath);
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void disabledInit()
    {
        driveBase.setOdometryEnabled(false);
        if (follower != null)
        {
            follower.cancel();
        }
    }
}
