package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;
import org.frc5687.rapidreact.subsystems.Climber.ClimberState;

public class PrepToClimb extends OutliersCommand{

    private Climber _climber;
    
    public PrepToClimb(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        // _climber.setStep(ClimberStep.PREP_TO_CLIMB);
        _climber.runControllers();
    }

    @Override
    public void initialize(){
        super.initialize();
        _climber.dropDriveSpeed();
        // _climber.setRockGoal(Constants.Climber.ROCKER_EXTENDED_POSITION);
        _climber.setStaGoal(Constants.Climber.STATIONARY_EXTENDED_POSITION);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return _climber.getStaAtGoal() || _climber.isStaArmUp() ;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stopStationaryArm();
        _climber.stopRockerArm();


    }
}
