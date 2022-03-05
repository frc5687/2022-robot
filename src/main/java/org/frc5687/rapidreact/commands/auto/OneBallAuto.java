package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.commands.Shoot;
import org.frc5687.rapidreact.commands.auto.DriveToPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuto extends SequentialCommandGroup{
    public OneBallAuto (
        DriveTrain driveTrain,
        Catapult catapult,
        //Intake intake,
        Pose2d destination,
        Rotation2d rotation
    ) {
        Rotation2d _rotation;
        Pose2d _newPose;

        _rotation = rotation;
        _newPose = new Pose2d(destination.getX(), destination.getY(), _rotation);
        addCommands(
            new Shoot(catapult),
            new DriveToPose(driveTrain, _newPose, 0.2)
        );
    }
}
