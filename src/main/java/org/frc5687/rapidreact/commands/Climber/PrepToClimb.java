package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberState;
import org.frc5687.rapidreact.subsystems.Climber.ClimberStep;

public class PrepToClimb extends OutliersCommand{

    private Climber _climber;
    
    public PrepToClimb(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        _climber.setStep(ClimberStep.PREP_TO_CLIMB);
        _climber.moveStaArm(ClimberState.EXTENDING_S_ARM, 20);
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.dropDriveSpeed();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return _climber.isStaArmUp();
    }
}
