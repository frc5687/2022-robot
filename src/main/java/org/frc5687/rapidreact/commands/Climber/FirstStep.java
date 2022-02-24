package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberStep;

public class FirstStep extends OutliersCommand{

    private Climber _climber;

    public FirstStep(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        _climber.setStep(ClimberStep.FIRST_STEP);
        _climber.retractStationaryArm(Constants.Climber.STATIONARY_RETRACTED_POSITION);
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
