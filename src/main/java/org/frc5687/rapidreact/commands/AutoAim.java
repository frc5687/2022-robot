package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DriveTrain;

public class AutoAim extends OutliersCommand {
    
    private DriveTrain _driveTrain;
    
    public AutoAim(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public void execute(){
        if (_driveTrain.hasTarget()) {
            _driveTrain.drive(0, 0, _driveTrain.getVisionControllerOutput(), true);
        }
        super.execute();
    }
    @Override
    public boolean isFinished() {
        if (_driveTrain.onTarget()) {
            _driveTrain.drive(0, 0, 0, true);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
