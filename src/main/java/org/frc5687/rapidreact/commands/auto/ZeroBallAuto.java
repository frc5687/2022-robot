package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * ZeroBall auto will drive robot out of tarmac.
 */
public class ZeroBallAuto extends SequentialCommandGroup {

    private final Pose2d _destination;
    private final Rotation2d _rotation;
    private final Pose2d _newPose;
    // private final Rotation2d _theta;

    public ZeroBallAuto(
        DriveTrain driveTrain,
        Pose2d destination,
        Rotation2d rotation
    ) {
        Double _velocity;

        // Was hardcoded for testing
        // _theta = new Rotation2d(0.0);
        //_destination = new Pose2d(1.0, 1.0, _theta);

        // Now get destination from caller
        _rotation = rotation;
        _destination = destination;
        _velocity = 0.2;
        _newPose = new Pose2d(_destination.getX(), _destination.getY(), _rotation);
        addCommands(
            new DriveToPose(driveTrain, _newPose, _velocity)
         );
    }

}
