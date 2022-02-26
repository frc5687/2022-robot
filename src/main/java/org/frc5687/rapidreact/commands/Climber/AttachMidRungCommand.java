package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

/**
 * Second step in climb.  We simply fully retract the stationary arm.
 */
public class AttachMidRungCommand extends OutliersCommand{

    private Climber _climber;
    
    public AttachMidRungCommand(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.setStaGoal(Constants.Climber.STATIONARY_RETRACTED_POSITION);
    }

    @Override
    public void execute(){
        super.execute();
        // _climber.setStep(ClimberStep.PREP_TO_CLIMB);
        _climber.runControllers();
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        return _climber.isStaAtGoal() || _climber.isStaArmDown();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stopStationaryArm();
    }
}
