/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.frc5687.rapidreact.config.Auto;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.00001;
    public static final double DEADBAND = 0.15;
    // Separate constants into individual inner classes corresponding
    // to subsystems or robot modes, to keep variable names shorter.

    /**
     * We use compass headings to reference swerve modules.
     *
     * When looking down at top of robot:
     *
     *          N
     *          |
     *      W -- -- E
     *          |
     *          S
     *
     * When robot is flipped over on its back:
     *
     *          N
     *          |
     *      E -- -- W
     *          |
     *          S
     */


    /** Constants for driving robot
     *
     * <p>These constants control how entire drivetrain works, including
     *
     * - what angle the wheels point,
     * - which direction the wheels turn,
     * - how sensitive the joystick is,
     * - maximum speed and acceleration
     */
    public static class DriveTrain {

        // Control
        public static final double DEADBAND = 0.2; // Avoid unintentional joystick movement

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.6223; // meters
        public static final double LENGTH = 0.6223; // meters

        /**
         * Swerve modules are on four corners of robot:
         *
         * NW  <- Width of robot ->  NE
         *             / \
         *              |
         *        Length of robot
         *              |
         *             \ /
         *  SW                       SE
         */

        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        /**
         *
         * Coordinate system is wacky:
         *
         * (X, Y):
         *   X is N or S, N is +
         *   Y is W or E, W is +
         *
         *   NW (+,+)  NE (+,-)
         *
         *   SW (-,+)  SE (-,-)
         *
         * We go counter-counter clockwise starting at NW of chassis:
         *
         *  NW, SW, SE, NE
         *
         * Note: when robot is flipped over, this is clockwise.
         *
         */

        // Position vectors for the swerve module kinematics
        // i.e. location of each swerve module from center of robot
        // see coordinate system above to understand signs of vector coordinates
        public static final Translation2d NORTH_WEST = new Translation2d( SWERVE_NS_POS, SWERVE_WE_POS ); // +,+
        public static final Translation2d SOUTH_WEST = new Translation2d( -SWERVE_NS_POS, SWERVE_WE_POS ); // -,+
        public static final Translation2d SOUTH_EAST = new Translation2d( -SWERVE_NS_POS, -SWERVE_WE_POS ); // -,-
        public static final Translation2d NORTH_EAST = new Translation2d( SWERVE_NS_POS, -SWERVE_WE_POS ); // +,-

        // Should be 0, but can correct for hardware error in swerve module headings here.
        public static final double NORTH_WEST_OFFSET = 0; // radians
        public static final double SOUTH_WEST_OFFSET = 0; // radians
        public static final double SOUTH_EAST_OFFSET = 0; // radians
        public static final double NORTH_EAST_OFFSET = 0; // radians

        // In case encoder is measuring rotation in the opposite direction we expect.
        public static final boolean NORTH_WEST_ENCODER_INVERTED = true;
        public static final boolean SOUTH_WEST_ENCODER_INVERTED = true;
        public static final boolean SOUTH_EAST_ENCODER_INVERTED = true;
        public static final boolean NORTH_EAST_ENCODER_INVERTED = true;

        // Maximum rates of motion
        public static final double MAX_MPS = 3.0; // Max speed of robot (m/s)
        public static final double MAX_MPS_DURING_CLIMB = MAX_MPS / 4; // Max speed of robot (m/s) during climb
        public static final double MAX_ANG_VEL = Math.PI * 1.5; // Max rotation rate of robot (rads/s)
        public static final double MAX_MPSS = 0.5; // Max acceleration of robot (m/s^2)

        // PID controller settings
        public static final double ANGLE_kP = 3.0;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;

        public static final double kP = 11.5;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double PROFILE_CONSTRAINT_VEL = 3.0 * Math.PI;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI;

        // Vision PID controller
        public static final double VISION_TOLERANCE = 0.01; // rads
        public static final double VISION_kP = 4.5;
        public static final double VISION_kI = 0.1;
        public static final double VISION_kD = 0.3;
        public static final double VISION_IRANGE = 1.0;

        public static final double POSITION_TOLERANCE = 0.02;
        public static final double ANGLE_TOLERANCE = 0.02;
    }

    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        public static final double kDt = 0.005;

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final int TIMEOUT = 200;
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 9.2;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.0508; // Meters with compression.
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 30.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
        // the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    }
    public static class UDPJetson {
        public static final int BUFFER = 1024;
    }

    public static class Catapult {
        public static final long DELAY = 100; // ms

        public static final boolean SPRING_MOTOR_INVERTED = false;
        public static final boolean WINCH_MOTOR_INVERTED = false;

        public static final int COUNTS_PER_REVOLUTION = 8196;

        public static final double GEAR_REDUCTION = 64.0;
        public static final double FALCON_GEAR_REDUCTION = 25.0;
//        public static final double GEAR_REDUCTION_VP = 50.0;

        public static final double BABY_NEO_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(11710);
        public static final double FALCON_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(6300);
        public static final int FALCON_TICKS = 2048;

        public static final double MAX_SPEED_WITH_GEAR_BOX = BABY_NEO_RAD_PER_SEC / GEAR_REDUCTION;
        public static final double SPRING_WINCH_DRUM_CIRCUMFERENCE = Units.inchesToMeters(0.75) * Math.PI; // meters
        public static final double ARM_WINCH_DRUM_CIRCUMFERENCE = Units.inchesToMeters(1.437) * Math.PI; // meters
        public static final int WINCH_CURRENT_LIMIT = 25; //amps

        // Physical characteristics
        public static final double POUND_PER_IN_TO_NEWTON_PER_METER = 0.0057101471627692;
        public static final double SPRING_RATE = 11.2 * POUND_PER_IN_TO_NEWTON_PER_METER; // Newtons per meter.
        public static final double ARM_LENGTH = Units.inchesToMeters(20.9);
        public static final double LEVER_ARM_LENGTH = Units.inchesToMeters(6.0);
        public static final double ARM_MASS = 1.2; //kg
        public static final double BALL_INERTIA = 0.0432;
        public static final double ROD_INERTIA = (1.0 / 3.0) * ARM_MASS * (ARM_LENGTH * ARM_LENGTH);
        public static final double INERTIA_OF_ARM = ROD_INERTIA + BALL_INERTIA;

        public static final double STOWED_ANGLE = Units.degreesToRadians(90 + 24.724);

        //Linear regression constants
        public static final double LINEAR_REGRESSION_SLOPE = 3.6073; // meters and radians
        public static final double LINEAR_REGRESSION_OFFSET = -0.0217; //meters and radians


        // Spring Linear actuator limits
        public static final double SPRING_BOTTOM_LIMIT = 0; //TODO: Real values

        // Winch actuator limits
        public static final double WINCH_BOTTOM_LIMIT = 0;

        // Controller Parameters
        public static final int MOTION_MAGIC_SLOT = 0;
        public static final double SPRING_kP = 1.001; // Always start with kP
        public static final double SPRING_kI = 0.000; // If possible avoid kI
        public static final double SPRING_kD = 0.0001; // 2nd Kd
        public static final double SPRING_kF = 0.0; // 2nd Kd
        public static final double SPRING_IZONE = 30.0;
        public static final double TICKS_TO_METERS = SPRING_WINCH_DRUM_CIRCUMFERENCE / (FALCON_TICKS * FALCON_GEAR_REDUCTION);
        public static final double CRUISE_VELOCITY = 50000; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double ACCELERATION = 90000; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double SPRING_TOLERANCE = 0.005; // m
        // winch
        public static final double WINCH_kP = 20.0; // Always start with kP
        public static final double WINCH_kI = 0.0; // If possible avoid kI
        public static final double WINCH_kD = 0.0; // 2nd Kd
//        public static final double MAX_WINCH_VELOCITY_MPS = ((NEO_RAD_PER_SEC / GEAR_REDUCTION)/ (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_VELOCITY_MPS = (MAX_SPEED_WITH_GEAR_BOX / (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_ACCELERATION_MPSS = MAX_WINCH_VELOCITY_MPS * 20.0; // heuristic.
        public static final double WINCH_TOLERANCE = 0.001; // m

        // DriveCatapult constants
        public static final double LOWERING_SPEED = 1.0;
        public static final double SPRING_ZERO_SPEED = -0.5;
        public static final double REMOVE_BALL_WINCH_GOAL = 0.1;
        public static final double REMOVE_BALL_SPRING_GOAL = 0.05;
        public static final double INITIAL_BALL_WINCH_GOAL = 0.255;
        public static final double INITIAL_BALL_SPRING_GOAL = 0.06;


    }

    public static class IntakeBlocker{
        public static final double DOWN_POSITION = 180;
        public static final double UP_POSITION = 50;
    }

    public static class Intake{
        public static final boolean INVERTED = false;
        public static final double ROLLER_IDLE_SPEED = 0.0;
        public static final double THE_BEANS = 0.8;
        public static final double GEAR_RATIO = 5.0;
        public static final double MAX_RPM = 6300 * GEAR_RATIO;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
    }
    public static class ColorSensor{
        public static final double COLOR_PROXIMITY_BUFFER = 130;
    }

    public static class Climber{
        public static final double FALCON_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(6300);
        public static final int FALCON_INTEGRATED_ENCODER_TICKS = 2048;
        public static final double GEAR_REDUCTION = 25.0;
        public static final double MAX_SPEED_WITH_GEAR_BOX = FALCON_RAD_PER_SEC / GEAR_REDUCTION;
        public static final double WINCH_DRUM_CIRCUMFERENCE = Units.inchesToMeters(1.12) * Math.PI; // meters
        public static final double ENCODER_RATIO = Units.inchesToMeters(26) / 498.0; 
        // public static final double ENCODER_RATIO  = ;
        public static final boolean STATIONARY_ARM_REVERSED = false;
        public static final int ARM_CURRENT_LIMIT = 50; // amps

        public static final boolean ROCKER_ARM_REVERSED = true;
        public static final int ROCKER_ARM_CURRENT_LIMIT = 40; // amps

        public static final int MOTION_MAGIC_SLOT = 0;
        public static final double kP = 0.8; // 40.0;
        public static final double kI = 0.0; // 15.0;
        public static final double kD = 0.05;
        public static final double kF = 0.0;
        public static final double MAX_VELOCITY_MPS = (MAX_SPEED_WITH_GEAR_BOX / (2 * Math.PI) * WINCH_DRUM_CIRCUMFERENCE);
        public static final double MAX_ACCELERATION_MPSS = MAX_VELOCITY_MPS * 50.0; // heuristic
        public static final double TOLERANCE = 0.005; // meters.

        public static final double CLIMB_DRIVE_SPEED = -1.0; //The speed of the drivetrain during climbing

        public static final int COUNTS_PER_REVOLUTION = 8196;


        // These distances are in METERS
        public static final double STATIONARY_RETRACTED_METERS = 0.0; 
        public static final double STATIONARY_CLOSE_METERS = 0.46;
        public static final double STATIONARY_EXTENDED_METERS = 0.6604; // TODO: Needs to be confirmed

        public static final double ROCKER_RETRACTED_METERS = 0.0;
        public static final double ROCKER_CLOSE_METERS = 0.4064;
        public static final double ROCKER_MID_METERS = 0.51; // 0.4826
        public static final double ROCKER_EXTENDED_METERS = 0.67; // TODO: Needs to be confirmed

        public static final double STATIONARY_ENCODER_CONVERSION_FACTOR = 0.05;
        public static final double ROCKER_ENCODER_CONVERSION_FACTOR = 0.05;
        public static final long ROCKER_PISTON_WAIT = 250;
        public static final long ROCKER_PISTON_SETTLE = 1000; // The time it takes the robot to rock from one side to the other
        public static final double ARM_STOW_SPEED = -0.3;

        public static final boolean ROCKER_ENCODER_INVERTED = false; // TODO: Needs to be calibrated via Phoenix tuner
        public static final boolean STATIONARY_ENCODER_INVERTED = false; // TODO: Needs to be calibrated via Phoenix tuner

        public static final double ARM_TOLERANCE = 0.005;
        public static final double ARM_IZONE = 30.0;

        public static final int MAX_STALL_CYCLES = 10;
        public static final double STALL_CURRENT = 20.0;
        public static final double STALL_MIN_RPM = 20.0;
        public static final double ARM_THRES_TIME = 0.5;
        public static final double CURRENT_THRES = 15;
        public static final boolean ROCKER_ENCODER_REVERSED = false; // TODO: Needs to be calibrated via Phoenix tuner
        public static final boolean STATIONARY_ENCODER_REVERSED = false; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double TICKS_TO_METERS = 0.00000149489; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double CRUISE_VELOCITY = 50000; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double ACCELERATION = 90000; // TODO: Needs to be calibrated via Phoenix tuner
    }

    public static class Camera{
        public static final int BRIGHTNESS = 60;
        public static final int FPS_LIMIT = 15;
        public static final boolean AUTO_EXPOSURE = true;
        public static final int EXPOSURE = 70;
        public static final int HEIGHT = 320;
        public static final int WIDTH = 240;
    }

    public static class Lights{
        public static final double BASE = 0.91; // Solid purple
        public static final double RIGHT_BALL = -0.01; // Strobe green
        public static final double WRONG_BALL = -0.35; // Strobe red 
        public static final double SHOOTING = -0.07; // Shooting
        public static final double INTAKE = -0.15; // Breath blue
        public static final double CLIMBING = -0.93; // Rainbow lava
        public static final double AIMING = -0.09; //Aiming
        public static final double RIGHT_BALL_ON_TARGET = 0.71; // Solid green
        public static final double WRONG_BALL_ON_TARGET = 0.61; // Solid red

    }
}