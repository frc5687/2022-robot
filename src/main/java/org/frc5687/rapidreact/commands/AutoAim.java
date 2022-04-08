/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DriveTrain;

public class AutoAim extends OutliersCommand {

    private DriveTrain _driveTrain;

    public AutoAim(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        if (_driveTrain.hasTarget()) {
            _driveTrain.vision(_driveTrain.getVisionHeading());
        }
    }

    @Override
    public boolean isFinished() {
        if (_driveTrain.onTarget()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
