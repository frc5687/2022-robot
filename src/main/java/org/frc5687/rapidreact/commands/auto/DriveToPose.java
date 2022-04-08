/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

/** Drive in autonomous mode (i.e., no OI control) to a field-relative pose. */
public class DriveToPose extends OutliersCommand {

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
    public DriveToPose(DriveTrain driveTrain, Pose2d pose, double velocity) {
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
         * <p>North = +X West = +Y East = -Y South = -X
         */
        _driveTrain.poseFollower(_destination, _velocity);
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.isAtPose(_destination)) {
            _driveTrain.drive(0, 0, 0);
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
