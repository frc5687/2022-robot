/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.OI;

import static org.frc5687.rapidreact.Constants.DriveTrain.*;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(3.0);
        _vyFilter = new SlewRateLimiter(3.0);
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
        double vx = _vxFilter.calculate(-_oi.getDriveY()) * (_driveTrain.getSpeed());
        double vy = _vyFilter.calculate(_oi.getDriveX()) * (_driveTrain.getSpeed());
        if (_oi.autoAim()) {
            _driveTrain.enableLimelight();
        } else {
            _driveTrain.disableLimelight();
        }
        metric("Robot heading", _driveTrain.getHeading().getRadians());
        double rot =
                (_oi.autoAim() && _driveTrain.hasTarget())
                        ? _driveTrain.getVisionControllerOutput()
                        : _oi.getRotationX() * MAX_ANG_VEL;
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
