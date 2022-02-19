package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.Servo;

public class ServoStop {
    
    private Servo stopper;
    private boolean feeding = false;

    public ServoStop(int channel){
        stopper = new Servo(channel);
    }

    public void lower(){
        //Lowers the blocking arm
        //Lets balls enter the catapult
        feeding = false;
        stopper.setAngle(180);
    }

    public void raise(){
        //Raises the blocking arm
        //Stops balls from enter the catapult
        feeding = true;
        stopper.setAngle(60);
    }

    public boolean getMode(){
        //Check if the indexer arm is feeding the catapult
        return feeding;
    }
}
