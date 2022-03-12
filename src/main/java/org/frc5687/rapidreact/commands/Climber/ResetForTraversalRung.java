package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberStep;

public class ResetForTraversalRung extends OutliersCommand{

    private Climber _climber;
    
    public ResetForTraversalRung(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated ResetForTraversalRung");
    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized ResetForTraversalRung");
        _climber.setStep(ClimberStep.RESET_TO_TRAVERSE);
        _climber.enableMetrics();
        _climber.setStaGoalMeters(Constants.Climber.STATIONARY_EXTENDED_METERS);
        enableMetrics();
    }

    @Override
    public void execute(){
        super.execute();
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        if (_climber.isStaAtGoal()) {
            _climber.setStep(Climber.ClimberStep.ATTACHED_HIGH);
            info("Finished ResetForTraversalRung");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stop();
        _climber.disableMetrics();
    }
}
