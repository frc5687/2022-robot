package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class Climb extends OutliersCommand{

    private Climber _climber;
    
    public Climb(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        //Climb up
        // _climber.moveStaArm(ClimberState.RETRACTING_S_ARM, 0.0);
        if(_climber.isStaArmDown()){
            //If we're all the way up rock out
            _climber.rockerOut();
            //_climber.moveRockArm(ClimberState.EXTENDING_R_ARM, 0.5);
        }
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return _climber.isStaArmDown();
    }
}
