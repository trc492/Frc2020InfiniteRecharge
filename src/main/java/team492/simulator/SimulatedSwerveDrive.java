package team492.simulator;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import trclib.TrcDriveBaseOdometry;
import trclib.TrcMotor;
import trclib.TrcOdometrySensor;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcUtil;

import java.util.Arrays;

public class SimulatedSwerveDrive extends TrcSwerveDriveBase
{
    private static TrcSwerveModule createModule()
    {
        return new TrcSwerveModule("module", new MockMotor(),
            new TrcPidMotor("mock", new MockMotor(), new TrcPidController("mock", null, 0, null), 0));
    }

    private static class MockMotor extends TrcMotor
    {
        public MockMotor()
        {
            super("mock");
        }

        @Override
        public double getMotorPosition()
        {
            return 0;
        }

        @Override
        public double getMotorVelocity()
        {
            return 0;
        }

        @Override
        public void setMotorPower(double value)
        {

        }

        @Override
        public boolean getInverted()
        {
            return false;
        }

        @Override
        public double getPower()
        {
            return 0;
        }

        @Override
        public boolean isLowerLimitSwitchActive()
        {
            return false;
        }

        @Override
        public boolean isUpperLimitSwitchActive()
        {
            return false;
        }

        @Override
        public void resetPosition(boolean hardware)
        {

        }

        @Override
        public void setBrakeModeEnabled(boolean enabled)
        {

        }

        @Override
        public void setInverted(boolean inverted)
        {

        }

        @Override
        public void setPositionSensorInverted(boolean inverted)
        {

        }

        @Override
        public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
        {

        }

        @Override
        public void setSoftLowerLimit(double position)
        {

        }

        @Override
        public void setSoftUpperLimit(double position)
        {

        }
    }

    public final double maxVel;
    public final double maxRotVel;
    private SwerveModuleState[] states;
    public final SwerveDriveKinematics kinematics;
    private Double lastTime;

    public SimulatedSwerveDrive(double maxVel, double maxRotVel)
    {
        super(createModule(), createModule(), createModule(), createModule(), 30, 30);
        this.maxVel = maxVel;
        this.maxRotVel = maxRotVel;
        double halfWidth = TrcUtil.METERS_PER_INCH * getWheelBaseWidth() / 2;
        double halfLength = TrcUtil.METERS_PER_INCH * getWheelBaseLength() / 2;
        kinematics = new SwerveDriveKinematics(new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth), new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth));
        stop();
        setDriveBaseOdometry(new SimulatedSwerveOdometry());
    }

    @Override
    public void stop(String owner, boolean resetSteer)
    {
        holonomicDrive(owner, 0, 0, 0);
    }

    @Override
    public void tankDrive(String owner, double leftPower, double rightPower, boolean inverted)
    {
        double rot = Math.toDegrees(maxVel * (leftPower - rightPower) / getWheelBaseWidth()) / maxRotVel;
        holonomicDrive(owner, 0, TrcUtil.average(leftPower, rightPower), rot);
    }

    @Override
    protected void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        double mag = TrcUtil.magnitude(x, y);
        if (mag > 1)
        {
            x /= mag;
            y /= mag;
        }
        rotation = clipMotorOutput(rotation);
        if (inverted)
        {
            x = -x;
            y = -y;
        }
        if (gyroAngle != 0)
        {
            double cosA = Math.cos(Math.toRadians(gyroAngle));
            double sinA = Math.sin(Math.toRadians(gyroAngle));
            double x1 = x * cosA - y * sinA;
            double y1 = x * sinA + y * cosA;
            x = x1;
            y = y1;
        }
        double xVel = x * maxVel;
        double yVel = y * maxVel;
        double rotVel = rotation * maxVel;

        states = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(yVel * TrcUtil.METERS_PER_INCH, -xVel * TrcUtil.METERS_PER_INCH,
                -Math.toRadians(rotVel)));
    }

    @Override
    public void setModuleVelocities(double[][] velocities)
    {
        states = Arrays.stream(velocities)
            .map(v -> new SwerveModuleState(v[0] * maxVel * TrcUtil.METERS_PER_INCH, Rotation2d.fromDegrees(-v[1])))
            .toArray(SwerveModuleState[]::new);
    }

    private class SimulatedSwerveOdometry extends TrcDriveBaseOdometry
    {
        public SimulatedSwerveOdometry()
        {
            super(new TrcOdometrySensor()
            {
                @Override
                public void resetOdometry(boolean resetHardware)
                {

                }

                @Override
                public Odometry getOdometry()
                {
                    return new Odometry(null, 0, 0, 0, 0, 0);
                }
            }, new TrcOdometrySensor()
            {
                @Override
                public void resetOdometry(boolean resetHardware)
                {

                }

                @Override
                public Odometry getOdometry()
                {
                    return new Odometry(null, 0, 0, 0, 0, 0);
                }
            });
        }

        @Override
        public synchronized void resetOdometry(boolean resetHardware, boolean resetAngle)
        {
            super.resetOdometry(resetHardware, resetAngle);
        }

        @Override
        public synchronized Odometry getOdometryDelta()
        {
            double currTime = TrcUtil.getCurrentTimeNanos() / 1e9;
            double dt = lastTime == null ? 0 : currTime - lastTime;
            lastTime = currTime;

            ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
            double xVel = -speeds.vyMetersPerSecond / TrcUtil.METERS_PER_INCH;
            double yVel = speeds.vxMetersPerSecond / TrcUtil.METERS_PER_INCH;
            double rotVel = Math.toDegrees(-speeds.omegaRadiansPerSecond);

            Odometry o = new Odometry();
            o.position.x = xVel * dt;
            o.position.y = yVel * dt;
            o.position.angle = rotVel * dt;
            o.velocity.x = xVel;
            o.velocity.y = yVel;
            o.velocity.angle = rotVel;
            return o;
        }
    }
}
