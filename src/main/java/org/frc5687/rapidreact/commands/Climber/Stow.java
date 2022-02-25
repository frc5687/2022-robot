package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class Stow extends OutliersCommand{

    private Climber _climber;
    
    public Stow(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        //Store the stationary arm
        _climber.zeroEncoder();
        //_climber.setStaSpeed(0.4);
        //if(_climber.isStaArmDown()){
        //    _climber.stop();
        //    _climber.zeroEncoder();
        //}
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public boolean isFinished(){
        return _climber.isStaArmDown();
    }
}
