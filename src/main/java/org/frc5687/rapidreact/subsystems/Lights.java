package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends OutliersSubsystem{

    private Spark _blinkens;
    private boolean _running;
    private Mode _mode = Mode.UNKNOW;
    
    public Lights(OutliersContainer container, int port) {
        super(container);
        _blinkens = new Spark(port);
        _running = true;
    }

    /**
     * Sets the color of the blinkens
     * @param color
     */
    public void set(double color){
        _blinkens.set(color);
    }

    /**
     * The basic setting for the lights
     */
    public void base(){
        _mode = Mode.BASE;
    }

    /**
     * Gets the current blinkens mode
     * @return
     */
    public Mode getMode(){
        return _mode;
    }

    /**
     * The robot has picked up the wrong ball
     */
    public void wrongBall(){
        _mode = Mode.WRONG_BALL;
    }

    /**
     * The robot has picked up the right ball
     */
    public void rightBall(){
        _mode = Mode.RIGHT_BALL;
    }

    /**
     * The robot is in shooting mode
     */
    public void shooting(){
        _mode = Mode.SHOOTING;
    }

    /**
     * Sets robot into aiming possible
     */
    public void aiming(){
        _mode = Mode.AIMING;
    }

    /**
     * The intake is deployed
     */
    public void intake(){
        _mode = Mode.INTAKE;
    }

    @Override
    public void periodic(){
        switch(getMode()){
            case UNKNOW:
                break;
            case BASE:
                set(Constants.Blinkens.SOLID_PURPLE);
                break;
            case WRONG_BALL:
                set(Constants.Blinkens.STROBE_WHITE);
                break;
            case RIGHT_BALL:
                set(Constants.Blinkens.STROBE_BLUE);
                break;
            case SHOOTING:
                set(Constants.Blinkens.SOLID_GREEN);
                break;
            case INTAKE:
                set(Constants.Blinkens.BEATING_BLUE);
                break;
            case CLIMBING:
                set(Constants.Blinkens.RAINBOW);
                break;
            case AIMING:
                set(Constants.Blinkens.SOLID_HOT_PINK);
                break;
        }
    }

    @Override
    public void updateDashboard() {
        metric("Blikens done", _running);
        metric("Blinken mode", _mode.toString());
    }

    /**
     * To store all the possible modes
     */
    public enum Mode{
        UNKNOW,
        BASE,
        RIGHT_BALL,
        WRONG_BALL,
        SHOOTING,
        INTAKE,
        CLIMBING,
        AIMING
    }
}
