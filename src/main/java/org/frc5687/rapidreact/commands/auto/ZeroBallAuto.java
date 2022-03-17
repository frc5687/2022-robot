package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.util.AutoChooser;

/** Drive robot out of tarmac. */
public class ZeroBallAuto extends SequentialCommandGroup {

    private Translation2d _translation;
    private Rotation2d _rotation;
    private Pose2d _destination;

    private Boolean _bypass;

    /** Construct a ZeroBall Auto SequentialCommandGroup */
    public ZeroBallAuto(
        DriveTrain driveTrain,
        AutoChooser.Position position
    ) {
        Double _velocity;

        _bypass = false;
        if (_bypass) {
            // Hard code for testing
            _translation = new Translation2d(1.0, 1.0);
            _rotation = new Rotation2d(0.0);
            _destination = new Pose2d(_translation, _rotation);
            _velocity = 0.2;
            addCommands(
                new DriveToPose(driveTrain, _destination, _velocity)
             );
             return;    
        }

        switch(position) {
            case First:
                driveTrain.resetOdometry(Auto.RobotPositions.FIRST);
                _translation = new Translation2d (
                    Auto.BallPositions.BALL_ONE.getX(),
                    Auto.BallPositions.BALL_ONE.getY()
                    );
                _rotation = Auto.Rotations.BALL_ONE_FROM_FIRST;
                break;
            case Second:
                driveTrain.resetOdometry(Auto.RobotPositions.SECOND);
                _translation = new Translation2d (
                    Auto.FieldPositions.ROBOT_POS_TWO_DEST.getX(),
                    Auto.FieldPositions.ROBOT_POS_TWO_DEST.getY()
                    );
                _rotation = new Rotation2d();
                break;
            case Third:
                driveTrain.resetOdometry(Auto.RobotPositions.THIRD);
                _translation = new Translation2d (
                    Auto.BallPositions.BALL_TWO.getX(),
                    Auto.BallPositions.BALL_TWO.getY()
                    );
                _rotation = Auto.Rotations.BALL_TWO_FROM_THIRD;
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

        // Now set destination and velocity
        _destination = new Pose2d(_translation, _rotation);
        _velocity = 0.2;
        addCommands(
            new DriveToPose(driveTrain, _destination, _velocity)
         );
    }

}
