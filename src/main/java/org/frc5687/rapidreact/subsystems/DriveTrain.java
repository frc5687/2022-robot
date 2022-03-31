/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

import static org.frc5687.rapidreact.Constants.EPSILON;


public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    public static final int NORTH_WEST = 0;
    public static final int SOUTH_WEST = 1;
    public static final int SOUTH_EAST = 2;
    public static final int NORTH_EAST = 3;

    private DiffSwerveModule _northWest;
    private DiffSwerveModule _southWest;
    private DiffSwerveModule _southEast;
    private DiffSwerveModule _northEast;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private double _PIDAngle;

    private boolean _isMoving;

    private AHRS _imu;
    private JetsonProxy _proxy;
//    private Limelight _limelight;
    private OI _oi;

    private HolonomicDriveController _controller;
    private ProfiledPIDController _angleController;
    private ProfiledPIDController _visionController;
    private ProfiledPIDController _ballController;

    private double _driveSpeed = Constants.DriveTrain.MAX_MPS;
    private boolean _useLimelight = false;

    private boolean _climbing = false;

    public DriveTrain(OutliersContainer container, OI oi, JetsonProxy proxy/*, Limelight limelight*/, AHRS imu) {
        super(container);
        _isMoving = false;
        try {
            _oi = oi;
            _proxy = proxy;
//            _limelight = limelight;
            _imu = imu;
            _northWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_WEST,
                            RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                            RobotMap.DIO.NORTH_WEST,
                            Constants.DriveTrain.NORTH_WEST_OFFSET,
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED);
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
            _northEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED);
            // NB: it matters which order these are defined
            _kinematics =
                    new SwerveDriveKinematics(
                            _northWest.getModulePosition(),
                            _southWest.getModulePosition(),
                            _southEast.getModulePosition(),
                            _northEast.getModulePosition()
                    );
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

            _visionController =
                    new ProfiledPIDController(
                            Constants.DriveTrain.VISION_kP,
                            Constants.DriveTrain.VISION_kI,
                            Constants.DriveTrain.VISION_kD,
                            new TrapezoidProfile.Constraints(
                                    Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
            _visionController.setIntegratorRange(-Constants.DriveTrain.VISION_IRANGE, Constants.DriveTrain.VISION_IRANGE);
            _visionController.setTolerance(Constants.DriveTrain.VISION_TOLERANCE);
            _ballController=
                    new ProfiledPIDController(
                            Constants.DriveTrain.BALL_VISION_kP,
                            Constants.DriveTrain.BALL_VISION_kI,
                            Constants.DriveTrain.BALL_VISION_kD,
                            new TrapezoidProfile.Constraints(
                                    Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
            _ballController.setIntegratorRange(-Constants.DriveTrain.BALL_VISION_IRANGE, Constants.DriveTrain.BALL_VISION_IRANGE);
            _ballController.setTolerance(Constants.DriveTrain.BALL_VISION_TOLERANCE);
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _northWest.periodic();
        _southWest.periodic();
        _southEast.periodic();
        _northEast.periodic();
    }

    public void dataPeriodic() {

    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _southWest.getState(),
                _southEast.getState(),
                _northEast.getState()
        );
    }

    @Override
    public void updateDashboard() {
        metric("Goal Distance From Top Plane", getDistanceToTarget());
        metric("Goal Distance From Points", Math.sqrt(
                (getTargetPosition()[0] * getTargetPosition()[0]) +
                (getTargetPosition()[1] * getTargetPosition()[1]) +
                (getTargetPosition()[2] * getTargetPosition()[2])
                ));
        metric("Goal Angle", getAngleToTarget());
        metric("Has goal", hasTarget());
        metric("Target vx", getTargetVelocity()[0]);
        metric("Target vy", getTargetVelocity()[1]);
        metric("Target vz", getTargetVelocity()[2]);
        metric("Target x", getTargetPosition()[0]);
        metric("Target y", getTargetPosition()[1]);
        metric("Target z", getTargetPosition()[2]);
        metric("Blue ball angle", getAngleToClosestBlueBall());
        metric("Red ball angle", getAngleToClosestRedBall());
        metric("Ball heading", getCorrectBallHeading().getRadians());

//        metric("NW/Encoder Angle", _northWest.getModuleAngle());
//        metric("SW/Encoder Angle", _southWest.getModuleAngle());
//        metric("SE/Encoder Angle", _southEast.getModuleAngle());
//        metric("NE/Encoder Angle", _northEast.getModuleAngle());
//
//        metric("SW/Predicted Angle", _southWest.getPredictedAzimuthAngle());
//
//        metric("SW/Encoder Azimuth Vel", _southWest.getAzimuthAngularVelocity());
//        metric("SW/Predicted Azimuth Vel", _southWest.getPredictedAzimuthAngularVelocity());
//
//        metric("SE/Encoder Wheel Vel", _southEast.getWheelVelocity());
//        metric("SE/Predicted Wheel Vel", _southEast.getPredictedWheelVelocity());
        metric("NW/Encoder Wheel Vel", _northWest.getWheelVelocity());
        metric("NW/Predicted Wheel Vel", _northWest.getPredictedWheelVelocity());


        metric("Odometry/x", getOdometryPose().getX());
        metric("Odometry/y", getOdometryPose().getY());
        metric("Estimated Pose/x", getEstimatedPose().getX());
        metric("Estimated Pose/y", getEstimatedPose().getY());
//        metric("Odometry/angle", getOdometryPose().getRotation().getDegrees());

    }

    public void setNorthEastModuleState(SwerveModuleState state) {
        _northEast.setIdealState(state);
    }

    public void setNorthWestModuleState(SwerveModuleState state) {
        _northWest.setIdealState(state);
    }

    public void setSouthEastModuleState(SwerveModuleState state) {
        _southEast.setIdealState(state);
    }

    public void setSouthWestModuleState(SwerveModuleState state) {
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
        if (Math.abs(vx) < EPSILON && Math.abs(vy) < EPSILON && Math.abs(omega) < EPSILON) {
            setNorthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            setSouthWestModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));
            setSouthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            setNorthEastModuleState(
                    new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
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
            setNorthWestModuleState(swerveModuleStates[NORTH_WEST]);
            setSouthWestModuleState(swerveModuleStates[SOUTH_WEST]);
            setSouthEastModuleState(swerveModuleStates[SOUTH_EAST]);
            setNorthEastModuleState(swerveModuleStates[NORTH_EAST]);
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
            setNorthWestModuleState(swerveModuleStates[NORTH_WEST]);
            setSouthWestModuleState(swerveModuleStates[SOUTH_WEST]);
            setSouthEastModuleState(swerveModuleStates[SOUTH_EAST]);
            setNorthEastModuleState(swerveModuleStates[NORTH_EAST]);
        }
        if (vx == 0 && vy == 0) {
            _isMoving = false;
        } else {
            _isMoving = true;
        }
    }
    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, Constants.DriveTrain.MAX_MPS);
    }
    public boolean hasTarget() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getTargetDistance() != -1;
        }
        return false;
    }

    public double getDistanceToTarget() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getTargetDistance();
        }
        return Double.NaN;
    }

    public double getAngleToTarget() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getTargetAngle();
        } //else if(_limelight.hasTarget()) {
           // return Units.degreesToRadians(_limelight.getYaw());
        //}
        return Double.NaN;
    }
    public double getAngleToClosestBlueBall() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getBlueBallYaw();
        }
        return Double.NaN;
    }
    public double getAngleToClosestRedBall() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getRedBallYaw();
        }
        return Double.NaN;
    }

    /**
     * Gets the Target x, y, z positions to the front of the catapult frame from the coprocessor.
     * @return array with 3 elements {x, y, z}
     */
    public double[] getTargetPosition() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().targetPosition();
        }
        return new double[] {0, 0, 0};
    }


    /**
     * Gets the Target velocities to the front of the catapult frame from the coprocessor.
     * @return array with 3 elements {vx, vy, vz}
     */
    public double[] getTargetVelocity() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().targetVelocity();
        }
        return new double[] {0, 0, 0};
    }

    /**
     * Gets the estimated robot pose from the coprocessor.
     * @return Pose2d
     */
    public Pose2d getEstimatedPose() {
        if (_proxy.getLatestFrame() != null) {
            return new Pose2d(_proxy.getLatestFrame().getEstimatedPose().getTranslation(), getHeading());
        }
        return new Pose2d();
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(getOdometryPose(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
        setNorthWestModuleState(moduleStates[NORTH_WEST]);
        setSouthWestModuleState(moduleStates[SOUTH_WEST]);
        setSouthEastModuleState(moduleStates[SOUTH_EAST]);
        setNorthEastModuleState(moduleStates[NORTH_EAST]);
    }

    public void poseFollower(Pose2d pose, double vel) {
        ChassisSpeeds adjustedSpeeds = _controller.calculate(getOdometryPose(), pose, vel, pose.getRotation());
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        setNorthWestModuleState(moduleStates[NORTH_WEST]);
        setSouthWestModuleState(moduleStates[SOUTH_WEST]);
        setSouthEastModuleState(moduleStates[SOUTH_EAST]);
        setNorthEastModuleState(moduleStates[NORTH_EAST]);
    }

    public void poseFollowerBallTracking(Pose2d pose, double vel) {
        Rotation2d rot = hasCorrectBall() ? getCorrectBallHeading() : pose.getRotation();
        ChassisSpeeds adjustedSpeeds = _controller.calculate(getOdometryPose(), pose, vel, rot);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        setNorthWestModuleState(moduleStates[NORTH_WEST]);
        setSouthWestModuleState(moduleStates[SOUTH_WEST]);
        setSouthEastModuleState(moduleStates[SOUTH_EAST]);
        setNorthEastModuleState(moduleStates[NORTH_EAST]);
    }

    public double getVisionControllerOutput(boolean ball) {
        if (ball) {
            return _ballController.calculate(getAngleCorrectBall());
        }
        return _visionController.calculate(-getAngleToTarget());
    }

    public boolean onTarget() {
        return Math.abs(getAngleToTarget()) < Constants.DriveTrain.VISION_TOLERANCE;
    }

    public boolean isMoving(){
        return _isMoving;
    }
    // checks if blue ball is visible
    public boolean hasBlueBall() {
        return getAngleToClosestBlueBall() != -99;
    }

    // checks if red ball is visible
    public boolean hasRedBall() {
        return getAngleToClosestRedBall() != -99;
    }

    // checks if correct ball is visible, this is decided by driver station.
    public boolean hasCorrectBall() {
        return (DriverStation.getAlliance() == DriverStation.Alliance.Red && hasRedBall()) ||
                (DriverStation.getAlliance() == DriverStation.Alliance.Blue && hasBlueBall());
    }

    public double getAngleCorrectBall() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return getAngleToClosestRedBall();
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return getAngleToClosestBlueBall();
        }
        return 0;
    }

    public Rotation2d getCorrectBallHeading() {
        return new Rotation2d(getHeading().getRadians() + getAngleCorrectBall());
    }

    public boolean isAtPose(Pose2d pose) {
        double diffX = getOdometryPose().getX() - pose.getX();
        double diffY = getOdometryPose().getY() - pose.getY();
        return (Math.abs(diffX) <= Constants.DriveTrain.POSITION_TOLERANCE) && (Math.abs(diffY) < Constants.DriveTrain.POSITION_TOLERANCE);
    }

    public boolean hasEstimatedPose() {
        if (_proxy != null) {
            return _proxy.getLatestFrame().hasEstimatedPose();
        }
        return false;
    }

    public Pose2d getOdometryPose() {
//        if (hasEstimatedPose()) {
//            return getEstimatedPose();
//        }
        return _odometry.getPoseMeters();
    }

    /** Reset position and gyroOffset of odometry
     * 
     * @param position is a Pose2d (Translation2d, Rotation2d)
     * 
     * <p> Translation2d resets odometry (X,Y) coordinates
     * 
     * <p> Rotation2d - gyroAngle = gyroOffset
     * 
     * <p> If Rotation2d <> gyroAngle, then robot heading will no longer equal IMU heading.
     */

    public void resetOdometry(Pose2d position) {
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(_reset, getHeading());
    }

    public void startModules() {
        _northWest.start();
        _southWest.start();
        _southEast.start();
        _northEast.start();
    }

    public double getSpeed() {
        return _driveSpeed;
    }

    /**
     * Called by the Climber to tell the drivetrain to slow down during a climb.
     * @param value
     */
    public void dropDriveSpeed(boolean value) {
        if (value) {
            _driveSpeed = Constants.DriveTrain.MAX_MPS_DURING_CLIMB;
            _climbing = true;
        } else {
            _driveSpeed = Constants.DriveTrain.MAX_MPS;
        }
    }

    public void turboDriveSpeed(boolean value) {
        if (_climbing) {
            _driveSpeed = Constants.DriveTrain.MAX_MPS_DURING_CLIMB;
        } else {
            _driveSpeed = value ? Constants.DriveTrain.MAX_MPS_TURBO : Constants.DriveTrain.MAX_MPS;
        }
    }

    public void enableLimelight() {
        _useLimelight = true;
//        _limelight.enableLEDs();
    }
    public void disableLimelight() {
        _useLimelight = false;
//        _limelight.disableLEDs();
    }
    public boolean useLimelight() {
        return _useLimelight;
    }
}