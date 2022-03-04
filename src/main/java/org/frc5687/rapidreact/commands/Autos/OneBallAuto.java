package org.frc5687.rapidreact.commands.Autos;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.Autos.DriveToPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuto extends SequentialCommandGroup{
    public OneBallAuto (
        DriveTrain driveTrain,
        //Catapult catapult,
        Pose2d destination,
        Rotation2d rotation
    ) {
        Rotation2d _rotation;
        Pose2d _newPose;

        _rotation = rotation;
        _newPose = new Pose2d(destination.getX(), destination.getY(), _rotation);
        addCommands(
            //shoot
            new DriveToPose(driveTrain, _newPose, new Rotation2d(0.0), 0.2)
        );
    }
}
