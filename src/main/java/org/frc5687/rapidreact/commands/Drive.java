/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.OI;

import static org.frc5687.rapidreact.Constants.DriveTrain.*;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;
    private final SlewRateLimiter _rotFilter;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(5.0);
        _vyFilter = new SlewRateLimiter(5.0);
        _rotFilter = new SlewRateLimiter(5.0);
        addRequirements(_driveTrain);
//        logMetrics("vx","vy");
//        enableMetrics();
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        double vx = _vxFilter.calculate(_oi.getDriveY()) * (_driveTrain.getSpeed());
        double vy = _vyFilter.calculate(_oi.getDriveX()) * (_driveTrain.getSpeed());
//        if (_oi.autoAim()) {
//            _driveTrain.enableLimelight();
//        } else {
//            _driveTrain.disableLimelight();
//        }
        double rot = 0;
        if (_oi.autoAim() && _driveTrain.hasTarget()) {
            rot = _driveTrain.getVisionControllerOutput(false);
        } else if (_oi.aimBall() &&
            ((DriverStation.getAlliance() == DriverStation.Alliance.Red && _driveTrain.hasRedBall()) ||
            (DriverStation.getAlliance() == DriverStation.Alliance.Blue && _driveTrain.hasBlueBall())))
        {
            rot = _driveTrain.getVisionControllerOutput(true);
        } else {
            rot = _rotFilter.calculate(_oi.getRotationX()) * MAX_ANG_VEL;
        }
        _driveTrain.drive(vx, vy, rot, true);

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
