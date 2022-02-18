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
        stopper.setAngle(Constants.IntakeBlocker.DOWN_POSITION);
    }

    public void raise(){
        //Raises the blocking arm
        //Stops balls from enter the catapult
        feeding = true;
        stopper.setAngle(Constants.IntakeBlocker.UP_POSITION);
    }

    public boolean getMode(){
        //Check if the indexer arm is feeding the catapult
        return feeding;
    }
}
