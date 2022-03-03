package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

/**
 * Dispatch class to run through the steps of the auto-climb one step at a time.
 * The intent is for this command to be triggered by a button, and each time the button is pressed
 * we advance the Climber's state machine and execute the next step.
 * Because we may need to retry or reverse the state machine, we'll actually maintain the state on
 * the climber itself. 
 */
public class SemiAutoClimb extends OutliersCommand {

    Climber _climber;

    public SemiAutoClimb(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        switch(_climber.getStep()) {
            case UNKNOWN:
                (new Stow(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.STOW);
                break;
            case STOW:
                (new PrepToClimb(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.PREP_TO_CLIMB);
                break;
            case PREP_TO_CLIMB:
                (new AttachMidRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_MID);
                break;
            case ATTACH_MID:
                (new AttachHighRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_HIGH);
                break;
            case ATTACH_HIGH:
                (new AttachTraversalRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_TRAVERSAL);
                break;
            case ATTACH_TRAVERSAL:
                _climber.setStep(Climber.ClimberStep.DONE);
                break;
            case DONE:
                break;
        }
    }     

    @Override
    public boolean isFinished() {
        return true;
    }



    
}
