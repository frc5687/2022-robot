package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class SecondStep extends OutliersCommand{

    private Climber _climber;
    
    public SecondStep(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }
}
