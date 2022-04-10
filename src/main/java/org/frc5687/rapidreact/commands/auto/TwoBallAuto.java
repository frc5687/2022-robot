/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.commands.auto;

import static org.frc5687.rapidreact.subsystems.Catapult.CatapultState.AIMING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.rapidreact.commands.AutoAim;
import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.DriveTrajectory;
import org.frc5687.rapidreact.commands.catapult.SetSetpoint;
import org.frc5687.rapidreact.commands.catapult.SetState;
import org.frc5687.rapidreact.commands.catapult.Shoot;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.TrajectoryManger;

/** Shoot first ball, taxi out of tarmac, intake second ball, shoot it */
public class TwoBallAuto extends SequentialCommandGroup {

    private Translation2d _translation;
    private Rotation2d _rotation;
    private Trajectory _trajectory;
    private Pose2d _destination;

    /** Construct a TwoBall Auto SequentialCommandGroup */
    public TwoBallAuto(
            DriveTrain driveTrain,
            Catapult catapult,
            Intake intake,
            Indexer indexer,
            AutoChooser.Position position,
            TrajectoryManger manger) {

        switch (position) {
            case First:
                driveTrain.resetOdometry(Auto.RobotPositions.FIRST);
                _trajectory =
                        manger.getTrajectory(TrajectoryManger.Trajectories.POS_ONE_TO_BALL_ONE);
                _rotation = Auto.Rotations.BALL_ONE_FROM_FIRST;
                break;
            case Second:
                driveTrain.resetOdometry(Auto.RobotPositions.SECOND);
                _trajectory =
                        manger.getTrajectory(TrajectoryManger.Trajectories.POS_TWO_TO_BALL_ONE);
                _rotation = new Rotation2d();
                break;
            case Third:
                driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                _trajectory =
                        manger.getTrajectory(TrajectoryManger.Trajectories.POS_THREE_TO_BALL_TWO);
                _rotation = Auto.Rotations.BALL_TWO_FROM_THIRD;
                break;
            case Fourth:
                driveTrain.resetOdometry(Auto.RobotPositions.FOURTH);
                _trajectory =
                        manger.getTrajectory(TrajectoryManger.Trajectories.POS_FOUR_TO_BALL_THREE);
                _rotation = Auto.Rotations.BALL_THREE_FROM_FOURTH;
                break;
            default:
                _translation =
                        new Translation2d(
                                driveTrain.getOdometryPose().getX(),
                                driveTrain.getOdometryPose().getY());
                _rotation = driveTrain.getOdometryPose().getRotation();
        }

        _destination = new Pose2d(_translation, _rotation);

        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new SetState(catapult, AIMING),
                                new SetSetpoint(catapult, CatapultSetpoint.TARMAC),
                                //                                                        new
                                // AutoAim(driveTrain),
                                new Shoot(catapult, indexer),
                                new DriveTrajectory(driveTrain, _trajectory, _rotation),
                                new WaitCommand(1)),
                        new AutoIntake(intake)),
                new AutoAim(driveTrain),
                new Shoot(catapult, indexer),
                new SetSetpoint(catapult, CatapultSetpoint.NONE));
    }
}
