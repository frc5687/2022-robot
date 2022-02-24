package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
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
        _climber.retractStationaryArm(Constants.Climber.STATIONARY_RETRACTED_POSITION);
        if(_climber.isStaArmDown()){
            _climber.stopClimb();
        }
    }

    @Override
    public boolean isFinished(){
        return _climber.isStaArmDown();
    }
}
