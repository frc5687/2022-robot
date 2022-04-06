/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import static org.frc5687.rapidreact.Constants.DriveTrain.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(4.0);
        _vyFilter = new SlewRateLimiter(4.0);
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
        _driveTrain.turboDriveSpeed(_oi.turbo());
        super.execute();
        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        double vx = _vxFilter.calculate(_oi.getDriveY());
        double vy = _vyFilter.calculate(_oi.getDriveX());
        double rot = _oi.getRotationX();

        if (_oi.autoAim() && _driveTrain.hasTarget()) {
            _driveTrain.vision(_driveTrain.getVisionHeading());
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
