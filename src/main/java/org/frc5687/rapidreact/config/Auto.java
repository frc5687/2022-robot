package org.frc5687.rapidreact.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto {
        
    public static class FieldPositions {
        public static Pose2d SAFE_BALL_THREE = new Pose2d(7.7, 0.8, new Rotation2d());
        public static Pose2d ROBOT_POS_ONE_DEST = new Pose2d(6.505, 5.685, new Rotation2d());
        public static Pose2d ROBOT_POS_TWO_DEST = new Pose2d(4.63, 3.65, new Rotation2d());
        public static Pose2d ROBOT_POS_THREE_DEST = new Pose2d(5.95, 1.42, new Rotation2d());
        public static Pose2d PARALLEL_PARK = new Pose2d(8.42, 0.4, new Rotation2d());

    }
    public static class BallPositions {
        public static Pose2d BALL_ONE = new Pose2d(4.755, 6.085, new Rotation2d());
        public static Pose2d BALL_TWO = new Pose2d(5.1, 1.77, new Rotation2d());
        public static Pose2d BALL_THREE = new Pose2d(7.7, 0.28, new Rotation2d());
    }

    public static class RobotPositions {
        public static Pose2d FIRST = new Pose2d(6.46, 5.45, new Rotation2d());
        public static Pose2d SECOND = new Pose2d(5.9, 3.93, new Rotation2d());
        public static Pose2d THIRD = new Pose2d(6.9, 2.5, new Rotation2d());
        public static Pose2d FOURTH = new Pose2d(8.25, 1.9, new Rotation2d());
    }

    public static class Rotations {
        public static Rotation2d BALL_THREE_FROM_FOURTH = new Rotation2d(0.5 * Math.PI);
    }

    public static final double DRIVETRAIN_POWER = 0.5;

    public static class StaticShots {
        public static double DEFAULT_WINCH = 0.24;
        public static double DEFAULT_SPRING = 0.065;

        public static double TARMAC_WINCH = 0.23;
        public static double TARMAC_SPRING = 0.055;

        // 3.15m
        public static double NEAR_WINCH = 0.24;
        public static double NEAR_SPRING = 0.065;

        // 5m
        public static double MID_WINCH = 0.312;
        public static double MID_SPRING = 0.079;

        // 6m
        public static double FAR_WINCH = 0.328;
        public static double FAR_SPRING = 0.0935;
    }
}
