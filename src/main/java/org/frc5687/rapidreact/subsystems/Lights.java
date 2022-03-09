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
     * The basic setting for the lights
     */
    public void base(){
        _blinkens.set(Constants.Blinkens.PULSING_RED);
        _mode = Mode.BASE;
    }

    /**
     * The robot has picked up the wrong ball
     */
    public void wrongBall(){
        _blinkens.set(Constants.Blinkens.SOLID_HOT_PINK);
        _mode = Mode.WRONG_BALL;
    }

    /**
     * The robot has picked up the right ball
     */
    public void rightBall(){
        _blinkens.set(Constants.Blinkens.BEATING_GREEN);
        _mode = Mode.RIGHT_BALL;
    }

    /**
     * The robot is in shooting mode
     */
    public void shooting(){
        _blinkens.set(Constants.Blinkens.BEATING_ORANGE);
        _mode = Mode.SHOOTING;
    }

    /**
     * The intake is deployed
     */
    public void intake(){
        _blinkens.set(Constants.Blinkens.RAINBOW);
        _mode = Mode.INTAKE;
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
        INTAKE
    }
}
