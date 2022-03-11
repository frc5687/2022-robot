package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.commands.OutliersCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RetryClimb extends OutliersCommand {

    private Climber _climber;
    public RetryClimb(Climber climber) {
        _climber = climber;
    }

    @Override
    public void initialize() {
        super.initialize();
        SequentialCommandGroup group = new SequentialCommandGroup(); 
        switch(_climber.getStep()) {
            case ATTACH_MID:
                group.addCommands(new ResetForMidRung(_climber));
                break;
            case ATTACHED_MID:
            case ATTACH_HIGH:
                group.addCommands(new ResetForHighRung(_climber));
                break;
            case ATTACHED_HIGH:
            case ATTACH_TRAVERSAL:
                group.addCommands(new ResetForTraversalRung(_climber));
                break;
            default:
                break;
        }
        group.schedule();
    }     

    @Override
    public boolean isFinished() {
        return true;
    }



    
}

