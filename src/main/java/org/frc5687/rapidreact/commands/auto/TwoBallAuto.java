package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.LowerCatapult;
import org.frc5687.rapidreact.commands.SetSetpoint;
import org.frc5687.rapidreact.commands.SetState;
import org.frc5687.rapidreact.commands.ShootSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;
import org.frc5687.rapidreact.commands.Shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.util.AutoChooser;

/** Shoot first ball, taix out of tarmc, intake second ball, shoot it */
public class TwoBallAuto extends SequentialCommandGroup {

    private Translation2d _translation;
    private Rotation2d _rotation;
    private Pose2d _destination;

    private Boolean _bypass;

    /** Construact a TwoBall Auto SequentialCommandGroup */
    public TwoBallAuto(
            DriveTrain driveTrain,
            Catapult catapult,
            Intake intake,
            AutoChooser.Position position
    ) {

        Double _velocity;

        _bypass = false;
        if (_bypass) {
            _translation = new Translation2d();
            _rotation = new Rotation2d();
            _destination = new Pose2d();
            _velocity = 0.2;
            addCommands(
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new Shoot(catapult),
                        new DriveToPose(driveTrain, _destination, _velocity),
                        new WaitCommand(1)
                    ),
                    new AutoIntake(intake)
                ),
                new SetSetpoint(catapult, CatapultSetpoint.MID),
                new Shoot(catapult),
                new SetSetpoint(catapult, CatapultSetpoint.NONE)

            );
        }

        switch(position) {
            case First:
                driveTrain.resetOdometry(Auto.RobotPositions.FIRST);
                _translation = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation = new Rotation2d();
                break;
            case Second:
                driveTrain.resetOdometry(Auto.RobotPositions.SECOND);
                _translation = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation = new Rotation2d();
                break;
            case Third:
                driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                _translation = new Translation2d (
                    Auto.BallPositions.BALL_TWO.getX(),
                    Auto.BallPositions.BALL_TWO.getY()
                    );
                _rotation = new Rotation2d();
                break;
            case Fourth:
                driveTrain.resetOdometry(Auto.RobotPositions.FOURTH);
                _translation = new Translation2d (
                    Auto.FieldPositions.SAFE_BALL_THREE.getX(),
                    Auto.FieldPositions.SAFE_BALL_THREE.getY()
                    );
                _rotation = Auto.Rotations.BALL_THREE_FROM_FOURTH;
                break;
            default:
                _translation = new Translation2d (
                    driveTrain.getOdometryPose().getX(),
                    driveTrain.getOdometryPose().getY()
                    );
                _rotation = driveTrain.getOdometryPose().getRotation();
        }

        _destination = new Pose2d(_translation, _rotation);
        _velocity = 0.2;

        addCommands(
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new Shoot(catapult),
                        new DriveToPose(driveTrain, _destination, _velocity),
                        new WaitCommand(1)
                    ),
                    new AutoIntake(intake)
                ),
                new SetSetpoint(catapult, CatapultSetpoint.MID),
                new Shoot(catapult),
                new SetSetpoint(catapult, CatapultSetpoint.NONE)

        );
    }
}
