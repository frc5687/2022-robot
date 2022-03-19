package org.frc5687.rapidreact.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Auto {
        
    public static class FieldPositions {
        public static Pose2d SAFE_BALL_THREE = new Pose2d(7.7, 0.8, new Rotation2d());
        public static Pose2d SAFE_BALL_FOUR = new Pose2d(1.55, 1.1, new Rotation2d());
        public static Pose2d ROBOT_POS_ONE_DEST = new Pose2d(6.505, 5.685, new Rotation2d());
        public static Pose2d ROBOT_POS_TWO_DEST = new Pose2d(4.63, 3.65, new Rotation2d());
        public static Pose2d ROBOT_POS_THREE_DEST = new Pose2d(5.95, 1.42, new Rotation2d());
        public static Pose2d PARALLEL_PARK = new Pose2d(8.42, 0.4, new Rotation2d());

    }

    public static class BallPositions {
        //new super accurate poses from amory.
        public static Pose2d BALL_ONE = new Pose2d(4.93, 6.223, new Rotation2d());
        public static Pose2d BALL_TWO = new Pose2d(5.035, 1.905, new Rotation2d());
        public static Pose2d BALL_THREE = new Pose2d(7.569, 0.305, new Rotation2d());
        public static Pose2d BALL_FOUR = new Pose2d(1.067, 1.14, new Rotation2d());
    }

    public static class RobotPositions {
        //new super accurate poses from amory.
        public static Pose2d FIRST = new Pose2d(6.604, 5.639, new Rotation2d());
        public static Pose2d SECOND = new Pose2d(5.9, 3.93, new Rotation2d());
        public static Pose2d THIRD = new Pose2d(6.724, 2.491, new Rotation2d());
        public static Pose2d FOURTH = new Pose2d(8.15, 1.877, new Rotation2d());
    }

    public static class Rotations {
        public static Rotation2d BALL_ONE_FROM_FIRST = new Rotation2d(-0.25 * Math.PI);
        public static Rotation2d BALL_TWO_FROM_THIRD = new Rotation2d(Units.degreesToRadians(20));
        public static Rotation2d BALL_TWO_FROM_BALL_THREE = new Rotation2d(Units.degreesToRadians(-35));
        public static Rotation2d BALL_THREE_FROM_FOURTH = new Rotation2d(0.5 * Math.PI);
        public static Rotation2d BALL_FOUR = new Rotation2d(Units.degreesToRadians(35));
    }

    public static final double DRIVETRAIN_POWER = 0.5;

    public static class StaticShots {
        public static double DEFAULT_WINCH = 0.24;
        public static double DEFAULT_SPRING = 0.065;

        public static double TARMAC_WINCH = 0.23;
        public static double TARMAC_SPRING = 0.055;

        // 3.15m
        public static double NEAR_WINCH = 0.24; // 0.24
        public static double NEAR_SPRING = 0.066; // 0.065

        // 4.35m
        public static double MID_WINCH = 0.315; //0.315
        public static double MID_SPRING = 0.076; //0.076

        // 6.5m
        public static double FAR_WINCH = 0.32; // 0.328
        public static double FAR_SPRING = 0.1; // 0.0935
    }
}
