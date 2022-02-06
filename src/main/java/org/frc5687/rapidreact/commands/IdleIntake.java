package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

public class IdleIntake extends OutliersCommand{
    
    Intake _intake;

    public IdleIntake(Intake intake){
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }
}
