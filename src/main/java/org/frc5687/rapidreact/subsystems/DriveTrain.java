/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.subsystems;

import static org.frc5687.rapidreact.Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS;
import static org.frc5687.rapidreact.Constants.DriveTrain.*;
import static org.frc5687.rapidreact.util.GeometryUtil.*;
import static org.frc5687.rapidreact.util.SwerveHeadingController.HeadingState.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.*;

public class DriveTrain extends OutliersSubsystem {
    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    private DiffSwerveModule _northWest, _southWest, _northEast, _southEast;
    private List<DiffSwerveModule> _modules;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private Vector2d _translationVector;

    private AHRS _imu;
    private HolonomicDriveController _controller;
    private SwerveHeadingController _headingController;
    private JetsonProxy _proxy;

    private double _driveSpeed = Constants.DriveTrain.MAX_MPS;
    private boolean _isMoving = false;
    private boolean _climbing = false;
    private boolean _lockHeading = false;

    public DriveTrain(OutliersContainer container, JetsonProxy proxy, AHRS imu) {
        super(container);
        try {
            _proxy = proxy;
            _imu = imu;
            _northWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_WEST,
                            RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                            RobotMap.DIO.NORTH_WEST,
                            Constants.DriveTrain.NORTH_WEST_OFFSET,
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED,
                            Constants.DriveTrain.CAN_BUS);
            _southWest =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_WEST,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                            RobotMap.DIO.SOUTH_WEST,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET,
                            Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED,
                            CAN_BUS);
            _southEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.SOUTH_EAST,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                            RobotMap.DIO.SOUTH_EAST,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET,
                            Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED,
                            CAN_BUS);
            _northEast =
                    new DiffSwerveModule(
                            Constants.DriveTrain.NORTH_EAST,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORTH_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED,
                            CAN_BUS);

            _modules = Arrays.asList(_northWest, _southWest, _southEast, _northEast);

            // NB: it matters which order these are defined
            _kinematics =
                    new SwerveDriveKinematics(
                            _northWest.getModulePosition(),
                            _southWest.getModulePosition(),
                            _southEast.getModulePosition(),
                            _northEast.getModulePosition());
            _odometry = new SwerveDriveOdometry(_kinematics, getHeading());

            _controller =
                    new HolonomicDriveController(
                            new PIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD),
                            new PIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD),
                            new ProfiledPIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD,
                                    new TrapezoidProfile.Constraints(
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_VEL,
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
            _headingController = new SwerveHeadingController(Constants.DriveTrain.kDt);
            _translationVector = new Vector2d();
            _isMoving = false;
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _modules.forEach(DiffSwerveModule::periodic);
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _southWest.getState(),
                _southEast.getState(),
                _northEast.getState());
    }

    @Override
    public void updateDashboard() {
        metric("Goal Distance From Top Plane", getDistanceToTarget());
        metric("Heading", getHeading().getDegrees());
        metric("Goal Angle", getAngleToTarget());
        metric("Has goal", hasTarget());
        //        metric("Target vx", getTargetVelocity()[0]);
        //        metric("Target vy", getTargetVelocity()[1]);
        //        metric("Target vz", getTargetVelocity()[2]);
        metric("Target x", getTargetPosition()[0]);
        metric("Target y", getTargetPosition()[1]);
        metric("Target z", getTargetPosition()[2]);

        //        metric("NW/Encoder Angle", _northWest.getModuleAngle());
        //        metric("SW/Encoder Angle", _southWest.getModuleAngle());
        //        metric("SE/Encoder Angle", _southEast.getModuleAngle());
        //        metric("NE/Encoder Angle", _northEast.getModuleAngle());
        //
        //        metric("SW/Predicted Angle", _southWest.getPredictedAzimuthAngle());
        //
        //        metric("SW/Encoder Azimuth Vel", _southWest.getAzimuthAngularVelocity());
        //        metric("SW/Predicted Azimuth Vel",
        // _southWest.getPredictedAzimuthAngularVelocity());
        //
        //        metric("SE/Encoder Wheel Vel", _southEast.getWheelVelocity());
        //        metric("SE/Predicted Wheel Vel", _southEast.getPredictedWheelVelocity());
        //        metric("NW/Encoder Wheel Vel", _northWest.getWheelVelocity());
        //        metric("NW/Predicted Wheel Vel", _northWest.getPredictedWheelVelocity());

        //        metric("Odometry/x", getOdometryPose().getX());
        //        metric("Odometry/y", getOdometryPose().getY());
        //        metric("Estimated Pose/x", getEstimatedPose().getX());
        //        metric("Estimated Pose/y", getEstimatedPose().getY());
        //        metric("Odometry/angle", getOdometryPose().getRotation().getDegrees());

    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.size(); module++) {
            _modules.get(module).setIdealState(states[module]);
        }
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
        metric("vx", vx);
        metric("vy", vy);
        metric("omega", omega);
        Vector2d translation = new Vector2d(vx, vy);

        double magnitude = translation.magnitude();

        if (Math.abs(getDistance(translation.direction(), getNearestPole(translation.direction())))
                < POLE_THRESHOLD) {
            translation =
                    rotationToVector(getNearestPole(translation.direction())).scale(magnitude);
        }

        if (magnitude < TRANSLATION_DEADBAND) {
            translation = new Vector2d();
            magnitude = 0;
        }

        Rotation2d direction = translation.direction();
        double scaledMagnitude = Math.pow(magnitude, POWER);
        translation =
                new Vector2d(
                        direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);

        if (translation.magnitude() > 1.0) {
            translation = translation.normalize();
        }

        omega = (Math.abs(omega) < ROTATION_DEADBAND) ? 0 : omega;
        // scale rotation
        omega = Math.pow(Math.abs(omega), POWER) * Math.signum(omega);

        translation = translation.scale(_driveSpeed);
        omega *= MAX_ANG_VEL;

        _translationVector = translation;

        if (omega == 0 && _translationVector.magnitude() != 0) {
            if (!_lockHeading) {
                stabilize();
            }
            _lockHeading = true;
        } else {
            _lockHeading = false;
            _headingController.setState(OFF);
        }

        double correctedOmega = omega + _headingController.getRotationCorrection(getHeading());
        metric("Drive X", _translationVector.x());
        metric("Drive Y", _translationVector.y());
        metric("Corrected Omega", correctedOmega);
        metric("Heading state", getCurrentHeadingState().name());
        metric("Target Heading", _headingController.getTargetHeading().getRadians());
        SwerveModuleState[] swerveModuleStates =
                _kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        _translationVector.x(),
                                        _translationVector.y(),
                                        correctedOmega,
                                        getHeading())
                                : new ChassisSpeeds(
                                        _translationVector.x(),
                                        _translationVector.y(),
                                        correctedOmega));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(swerveModuleStates);
        _isMoving = vx != 0 || vy != 0 || !(Math.abs(omega) < ROTATING_TOLERANCE);
    }

    public void snap(Rotation2d heading) {
        _headingController.setStabilizationHeading(heading);
    }

    public void stabilize() {
        _headingController.temporaryDisable();
    }

    public void vision(Rotation2d visionHeading) {
        _headingController.setVisionHeading(visionHeading);
    }

    public Rotation2d getVisionHeading() {
        return getHeading().plus(new Rotation2d(getAngleToTarget()));
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
            double dist = _proxy.getLatestFrame().getTargetDistance();
            if (dist > 1.5 && dist < 8) {
                return dist;
            }
            return -1;
        }
        return Double.NaN;
    }

    public double getAngleToTarget() {
        if (_proxy.getLatestFrame() != null) {
            return _proxy.getLatestFrame().getTargetAngle();
        }
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
     *
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
     *
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
     *
     * @return Pose2d
     */
    public Pose2d getEstimatedPose() {
        if (_proxy.getLatestFrame() != null) {
            return new Pose2d(
                    _proxy.getLatestFrame().getEstimatedPose().getTranslation(), getHeading());
        }
        return new Pose2d();
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds = _controller.calculate(getOdometryPose(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_SPEED_MPS);
        setModuleStates(moduleStates);
    }

    public void poseFollower(Pose2d pose, double vel) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(getOdometryPose(), pose, vel, pose.getRotation());
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        setModuleStates(moduleStates);
    }

    public void poseFollowerBallTracking(Pose2d pose, double vel) {
        Rotation2d rot = hasCorrectBall() ? getCorrectBallHeading() : pose.getRotation();
        ChassisSpeeds adjustedSpeeds = _controller.calculate(getOdometryPose(), pose, vel, rot);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        setModuleStates(moduleStates);
    }

    public SwerveHeadingController.HeadingState getCurrentHeadingState() {
        return _headingController.getHeadingState();
    }

    /**
     * calculates time to hit moving target
     *
     * @param position array of target
     * @param velocity array of target
     * @param speed exit velocity
     * @return lead time
     */
    public double calculateLeadTime(double[] position, double[] velocity, double speed) {
        double c0 = Helpers.dotProduct(position, position);
        double c1 = Helpers.dotProduct(position, velocity);
        double c2 = (speed * speed) * Helpers.dotProduct(velocity, velocity);
        double calculation = c1 * c1 + c2 * c0;
        double time = 0;
        if (calculation >= 0) {
            time = (c1 + Math.sqrt(calculation)) / c0;
            if (time < 0) {
                time = 0;
            }
        }
        return time;
    }

    /**
     * Calculates the new position with the target velocity
     *
     * @return Matrix<N3, N1> target position
     */
    public Matrix<N3, N1> leadTargetPosition(double[] position, double[] velocity, double time) {
        Vector<N3> pos = VecBuilder.fill(position[0], position[1], position[2]);
        Vector<N3> vel = VecBuilder.fill(velocity[0], velocity[1], velocity[2]);
        return pos.plus(vel.times(time));
    }

    public double calculateLeadAngle(Matrix<N3, N1> leadPosition) {
        // only care about 2d plane (x, y)
        Vector2d pos = new Vector2d(leadPosition.get(0, 0), leadPosition.get(1, 0));
        // unit vector of x
        Vector2d unitX = new Vector2d(1, 0);
        double angle;
        if (leadPosition.get(1, 0) < 0) {
            angle = -Math.acos(pos.dot(unitX) / (pos.magnitude() * unitX.magnitude()));
        } else {
            angle = Math.acos(pos.dot(unitX) / (pos.magnitude() * unitX.magnitude()));
        }
        return angle;
    }

    public boolean onTarget() {
        return Math.abs(getAngleToTarget()) < Constants.DriveTrain.VISION_TOLERANCE;
    }

    public boolean onAutoTarget() {
        return Math.abs(getAngleToTarget()) < Constants.DriveTrain.VISION_TOLERANCE + 0.01;
    }

    public boolean isMoving() {
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
        return (DriverStation.getAlliance() == DriverStation.Alliance.Red && hasRedBall())
                || (DriverStation.getAlliance() == DriverStation.Alliance.Blue && hasBlueBall());
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
        return (Math.abs(diffX) <= Constants.DriveTrain.POSITION_TOLERANCE)
                && (Math.abs(diffY) < Constants.DriveTrain.POSITION_TOLERANCE);
    }

    public boolean hasEstimatedPose() {
        if (_proxy != null) {
            return _proxy.getLatestFrame().hasEstimatedPose();
        }
        return false;
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Reset position and gyroOffset of odometry
     *
     * @param position is a Pose2d (Translation2d, Rotation2d)
     *     <p>Translation2d resets odometry (X,Y) coordinates
     *     <p>Rotation2d - gyroAngle = gyroOffset
     *     <p>If Rotation2d <> gyroAngle, then robot heading will no longer equal IMU heading.
     */
    public void resetOdometry(Pose2d position) {
        Translation2d _translation = position.getTranslation();
        Rotation2d _rotation = getHeading();
        Pose2d _reset = new Pose2d(_translation, _rotation);
        _odometry.resetPosition(_reset, getHeading());
    }

    public void startModules() {
        _modules.forEach(DiffSwerveModule::start);
    }

    public double getSpeed() {
        return _driveSpeed;
    }

    /**
     * Called by the Climber to tell the drivetrain to slow down during a climb.
     *
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

    public void setIsMoving(boolean isMoving) {
        _isMoving = isMoving;
    }
}
