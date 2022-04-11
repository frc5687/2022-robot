/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Vector2d;

public class DriveTrajectory extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Trajectory _trajectory;
    private Rotation2d _rot;
    private final Timer _timer;

    public DriveTrajectory(DriveTrain driveTrain, Trajectory trajectory, Rotation2d heading) {
        _driveTrain = driveTrain;
        _timer = new Timer();
        _trajectory = trajectory;
        _rot = heading;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setIsMoving(true);
        _driveTrain.disableHeadingController();
        _driveTrain.setControlState(DriveTrain.ControlState.TRAJECTORY);
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        Trajectory.State goal = _trajectory.sample(_timer.get());
        _driveTrain.updateSwerve(goal, _rot);
    }

    @Override
    public boolean isFinished() {
        if (_timer.get() >= _trajectory.getTotalTimeSeconds()) {
            _driveTrain.setIsMoving(false);
            //            _driveTrain.stabilize(_driveTrain.getHeading());
            _driveTrain.drive(0,0,0);
            _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
}
