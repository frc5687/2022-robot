package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frc5687.rapidreact.commands.AutoAim;
import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.DriveTrajectory;
import org.frc5687.rapidreact.commands.catapult.SetSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.commands.catapult.Shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.util.AutoChooser;

import java.awt.color.ICC_Profile;

public class FourBallAuto extends SequentialCommandGroup{

    /** Shoot first ball, taxi out of tarmac, intake second ball, shoot it */
    private Translation2d _translation1;
    private Translation2d _translation2;
    private Translation2d _translation3;
    private Rotation2d _rotation1;
    private Rotation2d _rotation2;
    private Rotation2d _rotation3;
    private Trajectory _trajectory;
    private Trajectory _trajectory1;
    private Trajectory _trajectory2;

    private Boolean _bypass;

    /** Construact a Four Ball Auto SequentialCommandGroup */
    public FourBallAuto(
            DriveTrain driveTrain,
            Catapult catapult,
            Intake intake,
            AutoChooser.Position position
    ) {
        /* No good way for a 4 ball auto from the First or Seconds position */
        /* Fourth position has two ways for 4 ball auto using the same sequence as Third position for simplicity*/
        switch(position) {
            case First:
                break;
            case Second:
                break;
            case Third:
                var config = driveTrain.getConfig();
                driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                _trajectory = TrajectoryGenerator.generateTrajectory(Auto.TrajectoryPoints.PositionThreeToBallTwo.waypoints, config);
                _trajectory1 = TrajectoryGenerator.generateTrajectory(Auto.TrajectoryPoints.BallTwoToBallFour.waypoints, config);
                _trajectory2 = TrajectoryGenerator.generateTrajectory(Auto.TrajectoryPoints.BallFourToFieldShot.waypoints, config);
                _rotation1 = Auto.Rotations.BALL_TWO_FROM_THIRD;
                _rotation2 = Auto.Rotations.BALL_FOUR;
                _rotation3 = Auto.Rotations.FAR_FIELD_SHOT;
                break;
            case Fourth:
                driveTrain.resetOdometry(Auto.RobotPositions.FOURTH);
                _translation1 = new Translation2d (
                        Auto.FieldPositions.SAFE_BALL_THREE.getX(),
                        Auto.FieldPositions.SAFE_BALL_THREE.getY()
                );
                _rotation1 = Auto.Rotations.BALL_THREE_FROM_FOURTH;
                _translation2 = new Translation2d(
                        Auto.FieldPositions.SAFE_BALL_FOUR.getX(),
                        Auto.FieldPositions.SAFE_BALL_FOUR.getY()
                );
                _rotation2 = Auto.Rotations.BALL_FOUR;
                _translation3 = new Translation2d(
                        Auto.FieldPositions.FAR_FIELD_SHOT.getX(),
                        Auto.FieldPositions.FAR_FIELD_SHOT.getY()
                );
                _rotation3 = Auto.Rotations.FAR_FIELD_SHOT;
                break;
            default:
                _translation1 = new Translation2d (
                        driveTrain.getOdometryPose().getX(),
                        driveTrain.getOdometryPose().getY()
                );
                _rotation1 = driveTrain.getOdometryPose().getRotation();
        }

        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new AutoAim(driveTrain),
                                new Shoot(catapult),
                                new DriveTrajectory(driveTrain, _trajectory, _rotation1),
                                new WaitCommand(0.3)
                        ),
                        new AutoIntake(intake)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new AutoAim(driveTrain),
                                new Shoot(catapult),
                                new DriveTrajectory(driveTrain, _trajectory1, _rotation2),
                                new WaitCommand(0.3)
                        ),
                        new AutoIntake(intake)
                ),
//                // wait a bit for the 2nd ball to roll in from human player station.
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new AutoIntake(intake)
                ),
                new DriveTrajectory(driveTrain, _trajectory2, _rotation3),
//            new SetSetpoint(catapult, CatapultSetpoint.FAR),
                new AutoAim(driveTrain),
                new Shoot(catapult),
                new Shoot(catapult),
                new SetSetpoint(catapult, CatapultSetpoint.NONE)
        );
    }
}
