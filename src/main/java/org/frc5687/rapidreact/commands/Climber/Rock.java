package org.frc5687.rapidreact.commands.Climber;

import com.fasterxml.jackson.databind.ser.std.AsArraySerializerBase;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

public class Rock extends OutliersCommand{

    private Climber _climber;
    
    public Rock(Climber climber){
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute(){
        super.execute();
        metric("Arm Forward", _climber.isArmForward());
        if(_climber.isArmForward()){
            _climber.reverse();
        }else{
            _climber.forward();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
