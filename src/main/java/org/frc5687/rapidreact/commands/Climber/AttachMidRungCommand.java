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
        info("Instantiated AttachMidRungCommand");
    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized AttachMidRungCommand");
        _climber.setStep(Climber.ClimberStep.ATTACH_MID);
        _climber.enableMetrics();
    }

    @Override
    public void execute(){
        super.execute();
        _climber.setStaSpeed(Constants.Climber.CLIMB_DRIVE_SPEED);
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        if (_climber.isStaArmDown()) {
            _climber.setStep(Climber.ClimberStep.ATTACHED_MID);
            info("Finished AttachMidRung");
            return true;
        };
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stop();
        _climber.disableMetrics();
    }
}
