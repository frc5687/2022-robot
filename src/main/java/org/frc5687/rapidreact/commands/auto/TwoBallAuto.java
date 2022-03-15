package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.frc5687.rapidreact.commands.AutoIntake;
import org.frc5687.rapidreact.commands.Catapult.SetSetpoint;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultSetpoint;
import org.frc5687.rapidreact.commands.Catapult.Shoot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(
            DriveTrain driveTrain,
            Catapult catapult,
            Intake intake,
            Pose2d destination,
            Rotation2d rotation
    ) {
        Rotation2d _rotation;
        Pose2d _newPose;
        _rotation = rotation;
        _newPose = new Pose2d(destination.getX(), destination.getY(), _rotation);
        addCommands(
                new ParallelDeadlineGroup( 
                    new SequentialCommandGroup(
                        new Shoot(catapult).withTimeout(5),
                        new DriveToPose(driveTrain, _newPose, 0.2),
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

