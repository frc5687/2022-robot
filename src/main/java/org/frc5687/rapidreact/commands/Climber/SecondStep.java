package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class SecondStep extends OutliersCommand{

    private Climber _climber;
    
    public SecondStep(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.forward();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }
}
