package org.frc5687.rapidreact.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class DriveToPoseAimBall extends OutliersCommand {

    private final Pose2d _destination;
    private final DriveTrain _driveTrain;
    // private final SlewRateLimiter _vxFilter;
    // private final SlewRateLimiter _vyFilter;

    private Double _velocity;

    /**
     * Create DriveAuto command
     *
     * @param driveTrain pass in from RobotContainer
     * @param pose xPos in meters, yPos in meters, theta in radians
     * @param velocity m/s
     */
    public DriveToPoseAimBall(
            DriveTrain driveTrain,
            Pose2d pose,
            double velocity
            ) {
        _driveTrain = driveTrain;
        _destination = pose;
        _velocity = velocity;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();

        /**
         * Based on observation, appears that
         *
         *             North = +X
         *  West = +Y              East = -Y
         *             South = -X
         */

        _driveTrain.poseFollowerBallTracking(_destination, _velocity);
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.isAtPose(_destination)) {
            _driveTrain.drive(0,0,0,true);
            info("DriveToPose finished.");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}