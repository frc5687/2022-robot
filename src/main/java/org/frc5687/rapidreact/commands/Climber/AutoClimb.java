package org.frc5687.rapidreact.commands.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

/**
 * Dispatch class to run through the steps of the auto-climb one step at a time.
 * The intent is for this command to be triggered by a button, and each time the button is pressed
 * we advance the Climber's state machine and execute the next step.
 * Because we may need to retry or reverse the state machine, we'll actually maintain the state on
 * the climber itself. 
 */
public class AutoClimb extends OutliersCommand {

    private Climber _climber;
    public AutoClimb(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        SequentialCommandGroup group = new SequentialCommandGroup(); 
        switch(_climber.getStep()) {
            case STOW:
            case STOWED:
            case UNKNOWN:
                group.addCommands(new PrepToClimb(_climber));
                break;
            case READY_TO_CLIMB:
            case ATTACH_MID:
                group.addCommands(new AttachMidRungCommand(_climber));
                group.addCommands(new AttachHighRungCommand(_climber));
                group.addCommands(new AttachTraversalRungCommand(_climber));
                break;
            case ATTACHED_MID:
            case ATTACH_HIGH:
                group.addCommands(new AttachHighRungCommand(_climber));
                group.addCommands(new AttachTraversalRungCommand(_climber));
                break;
            case ATTACHED_HIGH:
            case ATTACH_TRAVERSAL:
                group.addCommands(new AttachTraversalRungCommand(_climber));
                break;
            case ATTACHED_TRAVERSAL:
                _climber.setStep(Climber.ClimberStep.DONE);
                break;
            case DONE:
                break;
        }
        group.schedule();
    }     

    @Override
    public boolean isFinished() {
        return true;
    }



    
}
