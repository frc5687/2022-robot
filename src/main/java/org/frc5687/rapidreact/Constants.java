/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
        public static final double MAX_MPS = 2.5; // Max speed of robot (m/s)
        public static final double MAX_MPS_TURBO = 3.8; // Max speed of robot in turbo (m/s)
        public static final double MAX_MPS_DURING_CLIMB = MAX_MPS / 4; // Max speed of robot (m/s) during climb
        public static final double MAX_ANG_VEL = Math.PI * 1.5; // Max rotation rate of robot (rads/s)
        public static final double MAX_ANG_VEL_AIM = 2 * Math.PI; // Max rotation rate of robot (rads/s)
        public static final double MAX_MPSS = 2.5; // Max acceleration of robot (m/s^2)

        // PID controller settings
        public static final double ANGLE_kP = 2.3;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.6;
        public static final double PROFILE_CONSTRAINT_VEL = MAX_ANG_VEL;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI * 3.0;

        public static final double kP = 3.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;

        // Vision PID controller
        public static final double VISION_TOLERANCE = 0.04; // rads
        public static final double VISION_kP = 5.6;
        public static final double VISION_kI = 0.0;
        public static final double VISION_kD = 0.2;
        public static final double VISION_IRANGE = MAX_MPS * 2;
        public static final double MAX_ANG_VEL_VISION = Math.PI * 3; // Max rotation rate of robot (rads/s)
        public static final long VISION_LATENCY = 50;

        public static final double BALL_VISION_TOLERANCE = 0.040; // rads
        public static final double BALL_VISION_kP = 3.1;
        public static final double BALL_VISION_kI = 0.0;
        public static final double BALL_VISION_kD = 0.2;
        public static final double BALL_VISION_IRANGE = MAX_MPS * 2;

        public static final double POSITION_TOLERANCE = 0.03;
        public static final double ANGLE_TOLERANCE = 0.02;
        public static final double ROTATING_TOLERANCE = 0.3;
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
        public static final double WHEEL_RADIUS = 0.04515; // Meters with compression.
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 40.0;
        public static final double CURRENT_THRESHOLD = 40.0;
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
        // Module Constraints
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double MAX_ANGULAR_VELOCITY = FALCON_FREE_SPEED / GEAR_RATIO_STEER;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY * 10;
        // accel of wheel
        // ang vel
        public static final double MAX_MODULE_ACCELERATION = (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * 1.5;
        public static final double MAX_MODULE_JERK = MAX_MODULE_ACCELERATION * 10;
    }
    public static class UDPJetson {
        public static final int BUFFER = 1024;
    }

    public static class Catapult {
        public static final int CTRE_TIMEOUT = 20; // ms
        public static final int CTRE_FRAME_PERIOD = 10; // ms

        public static final long DELAY = 300; // ms
        public static final long LOWERING_DELAY = 100; // ms

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

        // Ball characteristics

        public static final double BALL_MASS = 0.27; // kg
        public static final double BALL_RADIUS = 0.24; // m of an inflated ball
        public static final double BALL_INERTIA = (2.0 / 3.0) * BALL_MASS * (BALL_RADIUS * BALL_RADIUS); // kg * m^2

        // Physical characteristics
        public static final double POUND_PER_IN_TO_NEWTON_PER_METER = 0.0057101471627692; // conversion factor
        public static final double SPRING_RATE = 11.2 * POUND_PER_IN_TO_NEWTON_PER_METER; // Newtons per meter.
        public static final double ARM_LENGTH = Units.inchesToMeters(20.9); // m
        public static final double LEVER_ARM_LENGTH = Units.inchesToMeters(6.0); // m
        public static final double ARM_MASS = 0.56; // kg
        public static final double ROD_INERTIA = (1.0 / 3.0) * ARM_MASS * (ARM_LENGTH * ARM_LENGTH); // kg * m^2
        public static final double INERTIA_OF_ARM = ROD_INERTIA + BALL_INERTIA; // kg * m^2

        public static final double STOWED_ANGLE = Units.degreesToRadians(24.724);

        //Linear regression constants
        public static final double LINEAR_REGRESSION_SLOPE = 3.6073; // meters and radians
        public static final double LINEAR_REGRESSION_OFFSET = -0.0217; //meters and radians

        // Spring Linear actuator limits
        public static final double SPRING_BOTTOM_LIMIT = 0.0245; //TODO: Real values

        // Winch actuator limits
        public static final double WINCH_BOTTOM_LIMIT = 0;

        // Controller Parameters
        public static final int MOTION_MAGIC_SLOT = 0;
        public static final double SPRING_kP = 1.5; // Followed with CTRE docs
        public static final double SPRING_kI = 0.001; // If possible avoid kI
        public static final double SPRING_kD = 10.0; // 2nd Kd
        public static final double SPRING_kF = 0.0;
        public static final double SPRING_IZONE = 50.0;
        public static final double TICKS_TO_METERS = SPRING_WINCH_DRUM_CIRCUMFERENCE / (FALCON_TICKS * FALCON_GEAR_REDUCTION);
        public static final double CRUISE_VELOCITY = 80000; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double ACCELERATION = 100000; // TODO: Needs to be calibrated via Phoenix tuner
        public static final double SPRING_TOLERANCE = 0.001; // m
        // winch
        public static final double WINCH_kP = 22.0; // Always start with kP
        public static final double WINCH_kI = 0.0; // If possible avoid kI
        public static final double WINCH_kD = 0.0; // 2nd Kd following
//        public static final double MAX_WINCH_VELOCITY_MPS = ((NEO_RAD_PER_SEC / GEAR_REDUCTION)/ (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_VELOCITY_MPS = (MAX_SPEED_WITH_GEAR_BOX / (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_ACCELERATION_MPSS = MAX_WINCH_VELOCITY_MPS * 50.0; // heuristic.
        public static final double WINCH_TOLERANCE = 0.001; // m

        // DriveCatapult constants
        public static final double LOWERING_SPEED = 1.0;
        public static final double SPRING_ZERO_SPEED = -0.5;

        // poly regression spring
        public static final double SPRING_CUBIC_COEFF = 0.006314395;
        public static final double SPRING_SQUARE_COEFF = -0.105778826;
        public static final double SPRING_LINEAR_COEFF = 0.592986356;
        public static final double SPRING_OFFSET_COEFF = -0.793641243;
        // poly regression winch
        public static final double WINCH_CUBIC_COEFF = -0.000068220;
        public static final double WINCH_SQUARE_COEFF = 0.001980322;
        public static final double WINCH_LINEAR_COEFF = -0.005533508;
        public static final double WINCH_OFFSET_COEFF = 0.064979245;

    }

    public static class Intake{
        public static final boolean INVERTED = false;
        public static final double ROLLER_IDLE_SPEED = 0.0;
        public static final double ROLLER_SLOW_SPEED = 0.4;
        public static final double THE_BEANS = 0.7;
        public static final double GEAR_RATIO = 5.0;
        public static final double MAX_RPM = 6300 * GEAR_RATIO;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final long ROLLER_DELAY = 1000; // ms
    }

    public static class ColorSensor{
        public static final double COLOR_PROXIMITY_BUFFER = 130;
    }

    public static class Indexer {
        public static final long NO_BALL_DELAY = 500; //ms
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

    public static class CANdle{
        public static double BRIGHTNESS = 0.5;
        public static int NUM_LED = 100;
        public static double SPEED = 0.7;

        // RGB Color Values
        public static int[] YELLOW = {255, 255, 0};
        public static int[] RED = {255, 0, 0};
        public static int[] GREEN = {0, 255, 0};
        public static int[] BLUE = {0, 0, 255};
        public static int[] CYAN = {0, 255, 255};
        public static int[] WHITE = {0, 0, 0};
        public static int[] PINK = {255, 105, 18};
        public static int[] GOLD = {212, 175, 55};
        public static int[] PURPLE = {128, 0, 128};
        public static int[] RUFOUS = {168, 28, 7};

        public static int[] ORANGE_RED = {255, 69, 0};
        public static int[] MAROON = {128, 0, 0};
    }

}