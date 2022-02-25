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
        // TODO Auto-generated method stub
        super.initialize();

        (new PrepToClimb(_climber)).schedule();
    }     



    
}
