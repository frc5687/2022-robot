package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class ArmUp extends OutliersCommand{

    private Climber _climber;
    
    public ArmUp(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        _climber.raiseSationaryArm();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stopClimb();
    }
}
