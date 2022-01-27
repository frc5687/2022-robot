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
    private DiffSwerveModule _frontRight;
    private DiffSwerveModule _frontLeft;
    private DiffSwerveModule _backRight;
    private DiffSwerveModule _backLeft;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odomerty;

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

            _frontRight =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_WEST,
                            RobotMap.CAN.TALONFX.FR_LEFT_FALCON,
                            RobotMap.CAN.TALONFX.FR_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_FR,
                            Constants.DriveTrain.NORTH_WEST_OFFSET);
            _frontLeft =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.FL_LEFT_FALCON,
                            RobotMap.CAN.TALONFX.FL_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_FL,
                            Constants.DriveTrain. NORTH_EAST_OFFSET);
            _backRight =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_WEST,
                            RobotMap.CAN.TALONFX.BR_LEFT_FALCON,
                            RobotMap.CAN.TALONFX.BR_RIGHT_FALCON,
                            RobotMap.DIO.ENCODER_BR,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET);
            _backLeft =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_EAST,
                            RobotMap.CAN.TALONFX.BL_RIGHT_FALCON,
                            RobotMap.CAN.TALONFX.BL_LEFT_FALCON,
                            RobotMap.DIO.ENCODER_BL,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET);
           _kinematics =
                    new SwerveDriveKinematics(
                            _frontLeft.getModulePosition(),
                            _frontRight.getModulePosition(),
                            _backLeft.getModulePosition(),
                            _backRight.getModulePosition());
            _odomerty = new SwerveDriveOdometry(_kinematics, getHeading());

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
        _frontRight.periodic();
        _frontLeft.periodic();
        _backRight.periodic();
        _backLeft.periodic();
    }

    @Override
    public void periodic() {
        _odomerty.update(
                getHeading(),
                _frontLeft.getState(),
                _frontRight.getState(),
                _backLeft.getState(),
                _backRight.getState());
    }

    @Override
    public void updateDashboard() {
        metric("BR/Encoder Angle", _backRight.getModuleAngle());
        metric("BL/Encoder Angle", _backLeft.getModuleAngle());
        metric("FL/Encoder Angle", _frontLeft.getModuleAngle());
        metric("FR/Encoder Angle", _frontRight.getModuleAngle());

        metric("BR/Predicted Angle", _backRight.getPredictedAzimuthAngle());

        metric("BR/Encoder Azimuth Vel", _backRight.getAzimuthAngularVelocity());
        metric("BR/Predicted Azimuth Vel", _backRight.getPredictedAzimuthAngularVelocity());

        metric("BR/Encoder Wheel Vel", _backRight.getWheelVelocity());
        metric("BR/Predicted Wheel Vel", _backRight.getPredictedWheelVelocity());

        metric("Odometry Pose", getOdometryPose().toString());
    }

    public void setFrontRightModuleState(SwerveModuleState state) {
        _frontRight.setIdealState(state);
    }

    public void setFrontLeftModuleState(SwerveModuleState state) {
        _frontLeft.setIdealState(state);
    }

    public void setBackLeftModuleState(SwerveModuleState state) {
        _backLeft.setIdealState(state);
    }

    public void setBackRightModuleState(SwerveModuleState state) {
        _backRight.setIdealState(state);
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
                    new SwerveModuleState(0, new Rotation2d(_frontRight.getModuleAngle())));
            setFrontLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_frontLeft.getModuleAngle())));
            setBackRightModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backRight.getModuleAngle())));
            setBackLeftModuleState(
                    new SwerveModuleState(0, new Rotation2d(_backLeft.getModuleAngle())));
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
                _controller.calculate(_odomerty.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
        setFrontLeftModuleState(moduleStates[0]);
        setFrontRightModuleState(moduleStates[1]);
        setBackLeftModuleState(moduleStates[2]);
        setBackRightModuleState(moduleStates[3]);
    }

    public Pose2d getOdometryPose() {
        return _odomerty.getPoseMeters();
    }

    public void startModules() {
        _frontRight.start();
        _frontLeft.start();
        _backLeft.start();
        _backRight.start();
    }
}
