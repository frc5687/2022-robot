package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class ClimberDown extends OutliersCommand{

    private Climber _climber;

    public ClimberDown(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        _climber.climb();
    }

    @Override
    public boolean isFinished(){
        return _climber.isArmDown();
    }
}
