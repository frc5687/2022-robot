package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import edu.wpi.first.wpilibj.Servo;

public class ServoStop {
    
    private Servo stopper;
    private boolean feeding = false;

    public ServoStop(){
        stopper = new Servo(9);
    }

    public void lower(){
        //Lowers the blocking arm
        //Lets balls enter the catapult
        feeding = false;
        stopper.setAngle(0);
    }

    public void raise(){
        //Raises the blocking arm
        //Stops balls from enter the catapult
        feeding = true;
        stopper.setAngle(120);
    }

    public boolean getMode(){
        //Check if the indexer arm is feeding the catapult
        return feeding;
    }
}
