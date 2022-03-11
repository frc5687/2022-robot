/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController _visionController;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(3.0);
        _vyFilter = new SlewRateLimiter(3.0);
        _visionController = new PIDController(
                VISION_kP,
                VISION_kI,
                VISION_kD
        );
        _visionController.setIntegratorRange(-VISION_IRANGE, VISION_IRANGE);
        _visionController.setTolerance(VISION_TOLERANCE);
        addRequirements(_driveTrain);
        logMetrics("vx","vy");
        enableMetrics();
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
        double vx = _vxFilter.calculate(-_oi.getDriveY()) * (_driveTrain.getSpeed() * _oi.getDriveZ());
        double vy = _vyFilter.calculate(_oi.getDriveX()) * (_driveTrain.getSpeed());
        metric("vx", vx);
        metric("vy", vy);
//        double rot = 0;
        metric("aim", _oi.autoAim());

        metric("has Target", _driveTrain.hasTarget());
        double rot =
                (_oi.autoAim() && _driveTrain.hasTarget())
                        ? _visionController.calculate(-_driveTrain.getAngleToTarget())
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
