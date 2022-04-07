package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberStep;

public class PrepToClimb extends OutliersCommand{

    private Climber _climber;
    
    public PrepToClimb(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated PrepToClimb Check");
    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized PrepToClimb");
        _climber.setClimbingLights();
        _climber.setStep(ClimberStep.PREP_TO_CLIMB);
        _climber.enableMetrics();
        _climber.dropDriveSpeed(true);
        _climber.setRockGoalMeters(Constants.Climber.ROCKER_EXTENDED_METERS);
        _climber.setStaGoalMeters(Constants.Climber.STATIONARY_EXTENDED_METERS);
        _climber.rockerIn();
        enableMetrics();
    }

    @Override
    public void execute(){
        super.execute();
        
        metric("Stationary/Speed", _climber.getStaSpeed());
        metric("Stationary/Position", _climber.getStaPositionMeters());
        metric("Stationary/Down", _climber.isStaArmDown());

        metric("Rocker/Speed", _climber.getRockSpeed());
        metric("Rocker/Position", _climber.getRockPositionMeters());
        metric("Rocker Cylinder", _climber.getRockerLabel());
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        if (_climber.isRockAtGoal()) {
            _climber.setStep(Climber.ClimberStep.READY_TO_CLIMB);
            info("Finished PrepToClimb");
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
