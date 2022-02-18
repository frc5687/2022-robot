package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.ServoStop;

public class Feed extends OutliersCommand{
    
    private ServoStop _servo;

    public Feed(ServoStop servo){
        _servo = servo;
    }

    @Override
    public void execute(){
        super.execute();
        if(_servo.getMode()){
            _servo.raise();
        }else{
            _servo.lower();
        }
    }
}