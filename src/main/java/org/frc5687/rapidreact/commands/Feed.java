package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.util.ServoStop;

public class Feed extends OutliersCommand{
    
    private ServoStop _servo;

    public Feed(ServoStop servo){
        _servo = servo;
    }

    @Override
    public void execute(){
        super.execute();
        _servo.lower();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _servo.raise();
    }
}