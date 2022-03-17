package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frc5687.rapidreact.commands.AutoAim;
import org.frc5687.rapidreact.commands.AutoIntake;
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

public class ThreeBallAuto extends SequentialCommandGroup{

/** Shoot first ball, taxi out of tarmac, intake second ball, shoot it */
    private Translation2d _translation1;
    private Translation2d _translation2;
    private Rotation2d _rotation1;
    private Rotation2d _rotation2;
    private Pose2d _destination1;
    private Pose2d _destination2;

    private Boolean _bypass;

    /** Construact a Three Ball Auto SequentialCommandGroup */
    public ThreeBallAuto(
            DriveTrain driveTrain,
            Catapult catapult,
            Intake intake,
            AutoChooser.Position position
    ) {

        Double _velocity;

        _bypass = false;
        if (_bypass) {
            _translation1 = new Translation2d();
            _translation2 = new Translation2d();
            _rotation1 = new Rotation2d();
            _rotation2 = new Rotation2d();
            _destination1 = new Pose2d();
            _destination2 = new Pose2d();
            _velocity = 0.2;
            addCommands(
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new AutoAim(driveTrain),
                        new Shoot(catapult),
                        new DriveToPose(driveTrain, _destination1, _velocity),
                        new WaitCommand(1)
                    ),
                    new WaitCommand(0)
//                    new AutoIntake(intake)
                ),
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new SetSetpoint(catapult, CatapultSetpoint.MID),
                        new AutoAim(driveTrain),
                        new Shoot(catapult),
                        new DriveToPose(driveTrain, _destination2, _velocity),
                        new WaitCommand(1)
                    ),
                        new WaitCommand(0)
//                    new AutoIntake(intake)
                ),
                new SetSetpoint(catapult, CatapultSetpoint.FAR),
                new AutoAim(driveTrain),
                new Shoot(catapult),
                new SetSetpoint(catapult, CatapultSetpoint.NONE)
            );
        }

        switch(position) {
            case First:
                driveTrain.resetOdometry(Auto.RobotPositions.FIRST);
                _translation1 = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation1 = Auto.Rotations.BALL_ONE_FROM_FIRST;
                _translation2 = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation2 = Auto.Rotations.BALL_ONE_FROM_FIRST;
                break;
            case Second:
                driveTrain.resetOdometry(Auto.RobotPositions.SECOND);
                _translation1 = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation1 = new Rotation2d();
                _translation2 = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation2 = Auto.Rotations.BALL_ONE_FROM_FIRST;
                break;
            case Third:
                driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                _translation1 = new Translation2d (
                    Auto.BallPositions.BALL_TWO.getX(),
                    Auto.BallPositions.BALL_TWO.getY()
                    );
                _rotation1 = Auto.Rotations.BALL_TWO_FROM_THIRD;
                _translation2 = new Translation2d (
                    Auto.FieldPositions.SAFE_BALL_FOUR.getX(),
                    Auto.FieldPositions.SAFE_BALL_FOUR.getY()
                    );
                _rotation2 = Auto.Rotations.BALL_FOUR;
                break;
            case Fourth:
                driveTrain.resetOdometry(Auto.RobotPositions.FOURTH);
                _translation1 = new Translation2d (
                    Auto.FieldPositions.SAFE_BALL_THREE.getX(),
                    Auto.FieldPositions.SAFE_BALL_THREE.getY()
                    );
                _rotation1 = Auto.Rotations.BALL_THREE_FROM_FOURTH;
                _translation2 = new Translation2d (
                    Auto.BallPositions.BALL_TWO.getX(),
                    Auto.BallPositions.BALL_TWO.getY()
                    );
                _rotation2 = Auto.Rotations.BALL_TWO_FROM_BALL_THREE;
                break;
            default:
                _translation1 = new Translation2d (
                    driveTrain.getOdometryPose().getX(),
                    driveTrain.getOdometryPose().getY()
                    );
                _rotation1 = driveTrain.getOdometryPose().getRotation();
        }

        _destination1 = new Pose2d(_translation1, _rotation1);
        _destination2 = new Pose2d(_translation2, _rotation2);
        _velocity = 0.2;

        addCommands(
            new ParallelDeadlineGroup( 
                new SequentialCommandGroup(
                    new AutoAim(driveTrain),
                    new Shoot(catapult),
                    new DriveToPose(driveTrain, _destination1, _velocity),
                    new WaitCommand(1)
                ),
                    new WaitCommand(0)
//                new AutoIntake(intake)
            ),
            new ParallelDeadlineGroup( 
                new SequentialCommandGroup(
                    new SetSetpoint(catapult, CatapultSetpoint.MID),
                    new AutoAim(driveTrain),
                    new Shoot(catapult),
                    new DriveToPose(driveTrain, _destination2, _velocity),
                    new WaitCommand(1)
                ),
                    new WaitCommand(0)
//                new AutoIntake(intake)
            ),
            new SetSetpoint(catapult, CatapultSetpoint.FAR),
            new AutoAim(driveTrain),
            new Shoot(catapult),
            new SetSetpoint(catapult, CatapultSetpoint.NONE)
        );
    }
}
