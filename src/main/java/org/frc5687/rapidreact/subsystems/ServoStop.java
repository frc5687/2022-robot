package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import edu.wpi.first.wpilibj.Servo;

public class ServoStop {
    
    private Servo stopper;

    public ServoStop(){
        stopper = new Servo(9);
    }

    public void lower(){
        //Lowers the blocking arm
        //Lets balls enter the catapult
        stopper.setAngle(Constants.IntakeBlocker.DOWN_POSITION);
    }

    public void raise(){
        //Raises the blocking arm
        //Stops balls from enter the catapult
        stopper.setAngle(Constants.IntakeBlocker.UP_POSITION);
    }
}
