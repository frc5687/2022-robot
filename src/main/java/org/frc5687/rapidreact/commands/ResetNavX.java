package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DriveTrain;

public class ResetNavX extends OutliersCommand {

    private DriveTrain _driveTrain;

    public ResetNavX(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }
    
    @Override
    public void initialize() {
        _driveTrain.resetYaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
