package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.Servo;

public class ServoStop {
    
    private static Servo stopper;
    private static boolean feeding = false;

    public ServoStop(int channel){
        stopper = new Servo(channel);
    }

    public static void lower(){
        //Lowers the blocking arm
        //Lets balls enter the catapult
        feeding = false;
        stopper.setAngle(70);
    }

    public static void raise(){
        //Raises the blocking arm
        //Stops balls from enter the catapult
        feeding = true;
        stopper.setAngle(120);
    }

    public static boolean getMode(){
        //Check if the indexer arm is feeding the catapult
        return feeding;
    }
}
