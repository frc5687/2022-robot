package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DriveTrain;

public class AutoAim extends OutliersCommand{
    
    private DriveTrain _driveTrain;
    
    public AutoAim(DriveTrain driveTrain){
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void initialize(){
        super.initialize();
    }
}
