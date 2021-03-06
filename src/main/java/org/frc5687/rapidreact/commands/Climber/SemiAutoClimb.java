package org.frc5687.rapidreact.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.DriveTrain;

/**
 * Dispatch class to run through the steps of the auto-climb one step at a time.
 * The intent is for this command to be triggered by a button, and each time the button is pressed
 * we advance the Climber's state machine and execute the next step.
 * Because we may need to retry or reverse the state machine, we'll actually maintain the state on
 * the climber itself. 
 */
public class SemiAutoClimb extends OutliersCommand {

    private Climber _climber;
    public SemiAutoClimb(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        switch(_climber.getStep()) {
            case UNKNOWN:
            case STOW:
                (new Stow(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.STOW);
                break;
            case STOWED:
            case PREP_TO_CLIMB:
                (new PrepToClimb(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.PREP_TO_CLIMB);
                break;
            case READY_TO_CLIMB:
            case ATTACH_MID:
                (new AttachMidRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_MID);
                break;
            case ATTACHED_MID:
            case ATTACH_HIGH:
                (new AttachHighRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_HIGH);
                break;
            case ATTACHED_HIGH:
            case ATTACH_TRAVERSAL:
                (new AttachTraversalRungCommand(_climber)).schedule();
                _climber.setStep(Climber.ClimberStep.ATTACH_TRAVERSAL);
                break;
            case ATTACHED_TRAVERSAL:
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
