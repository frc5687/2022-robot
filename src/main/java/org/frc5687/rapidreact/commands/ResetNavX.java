package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DriveTrain;

public class ResetNavX extends OutliersCommand{
    
    private DriveTrain _driveTrain; 
    
    public ResetNavX(DriveTrain driveTrain){
        _driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize(){
        super.initialize();
        _driveTrain.resetNavX();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }
}
