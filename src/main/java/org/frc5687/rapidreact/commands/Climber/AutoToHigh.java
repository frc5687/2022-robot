package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoToHigh extends OutliersCommand{
    
    private Climber _climber;

    public AutoToHigh(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize(){
        super.initialize();
        SequentialCommandGroup group = new SequentialCommandGroup();
        switch(_climber.getStep()){
            case UNKNOWN:
            case STOW:
            case STOWED:
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
