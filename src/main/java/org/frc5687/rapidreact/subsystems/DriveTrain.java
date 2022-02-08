/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.util.OutliersContainer;

public class DriveTrain extends OutliersSubsystem {
    private DiffSwerveModule _northWest;
    private DiffSwerveModule _northEast;
    private DiffSwerveModule _southWest;
    private DiffSwerveModule _southEast;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private double _PIDAngle;

    private AHRS _imu;
    private OI _oi;

    private HolonomicDriveController _controller;
    private ProfiledPIDController _angleController;

    public DriveTrain(OutliersContainer container, OI oi, AHRS imu) {
        super(container);
        try {
            _oi = oi;
            _imu = imu;

            _northWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_WEST,
                            RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                            RobotMap.DIO.NORTH_WEST,
                            Constants.DriveTrain.NORTH_WEST_OFFSET,
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED);
            _northEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORHT_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED);
            _southWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_WEST,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                            RobotMap.DIO.SOUTH_WEST,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET,
                            Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED);
            _southEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_EAST,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                            RobotMap.DIO.SOUTH_EAST,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET,
                            Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED);
           _kinematics =
                    new SwerveDriveKinematics(
                            _northEast.getModulePosition(),
                            _northWest.getModulePosition(),
                            _southEast.getModulePosition(),
                            _southWest.getModulePosition());
            _odometry = new SwerveDriveOdometry(_kinematics, getHeading());

            _controller =
                    new HolonomicDriveController(
                            new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                            new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                            new ProfiledPIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD,
                                    new TrapezoidProfile.Constraints(
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
            _angleController =
                    new ProfiledPIDController(
                            Constants.DriveTrain.ANGLE_kP,
                            Constants.DriveTrain.ANGLE_kI,
                            Constants.DriveTrain.ANGLE_kD,
                            new TrapezoidProfile.Constraints(
                                    Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
            _angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _northWest.periodic();
        _northEast.periodic();
        _southWest.periodic();
        _southEast.periodic();
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northEast.getState(),
                _northWest.getState(),
                _southEast.getState(),
                _southWest.getState());
    }

    @Override
    public void updateDashboard() {
        metric("SW/Encoder Angle", _southWest.getModuleAngle());
        metric("SE/Encoder Angle", _southEast.getModuleAngle());
        metric("NE/Encoder Angle", _northEast.getModuleAngle());
        metric("NW/Encoder Angle", _northWest.getModuleAngle());

        metric("SW/Predicted Angle", _southWest.getPredictedAzimuthAngle());

        metric("SW/Encoder Azimuth Vel", _southWest.getAzimuthAngularVelocity());
        metric("SW/Predicted Azimuth Vel", _southWest.getPredictedAzimuthAngularVelocity());

        metric("SW/Encoder Wheel Vel", _southWest.getWheelVelocity());
        metric("SW/Predicted Wheel Vel", _southWest.getPredictedWheelVelocity());
        Pose2d odometry=getOdometryPose();
        metric("Odometry/x",odometry.getX());
        metric("Odometry/y",odometry.getY());
        metric("Odometry/angle",odometry.getRotation().getDegrees());
        metric("Odometry/Pose", getOdometryPose().toString());
    }

    public void setFrontRightModuleState(SwerveModuleState state) {
        _northWest.setIdealState(state);
    }

    public void setFrontLeftModuleState(SwerveModuleState state) {
        _northEast.setIdealState(state);
    }

    public void setBackLeftModuleState(SwerveModuleState state) {
        _southEast.setIdealState(state);
    }

    public void setBackRightModuleState(SwerveModuleState state) {
        _southWest.setIdealState(state);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    // yaw is negative to follow wpi coordinate system.
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }

    public void resetYaw() {
        _imu.reset();
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, omega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param omega angular velocity (rotating speed)
     * @param fieldRelative forward is always forward no mater orientation of robot.
     */
    public void drive(double vx, double vy, double omega, boolean fieldRelative) {
        if (Math.abs(vx) < Constants.DriveTrain.DEADBAND && Math.abs(vy) < Constants.DriveTrain.DEADBAND && Math.abs(omega) < Constants.DriveTrain.DEADBAND) {
            setFrontRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            setFrontLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
            setBackRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));
            setBackLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(omega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, getHeading())
                                    : new ChassisSpeeds(vx, vy, omega));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
            setFrontLeftModuleState(swerveModuleStates[0]);
            setFrontRightModuleState(swerveModuleStates[1]);
            setBackLeftModuleState(swerveModuleStates[2]);
            setBackRightModuleState(swerveModuleStates[3]);
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx,
                                    vy,
                                    _angleController.calculate(
                                            getHeading().getRadians(), _PIDAngle),
                                    new Rotation2d(_PIDAngle)));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
            setFrontLeftModuleState(swerveModuleStates[0]);
            setFrontRightModuleState(swerveModuleStates[1]);
            setBackLeftModuleState(swerveModuleStates[2]);
            setBackRightModuleState(swerveModuleStates[3]);
        }
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, Constants.DriveTrain.MAX_MPS);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(_odometry.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
        setFrontLeftModuleState(moduleStates[0]);
        setFrontRightModuleState(moduleStates[1]);
        setBackLeftModuleState(moduleStates[2]);
        setBackRightModuleState(moduleStates[3]);
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public void startModules() {
        _northWest.start();
        _northEast.start();
        _southEast.start();
        _southWest.start();
    }
}
