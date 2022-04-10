/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class DriveToPoseAimBall extends OutliersCommand {

    private final Pose2d _destination;
    private final DriveTrain _driveTrain;

    /**
     * Create DriveAuto command
     *
     * @param driveTrain pass in from RobotContainer
     * @param pose xPos in meters, yPos in meters, theta in radians
     */
    public DriveToPoseAimBall(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _destination = pose;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
        _driveTrain.setPoseGoal(_destination);
        _driveTrain.setControlState(DriveTrain.ControlState.POSITION);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.isAtPose(_destination)) {
            _driveTrain.setControlState(DriveTrain.ControlState.NEUTRAL);
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
