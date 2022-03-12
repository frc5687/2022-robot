package org.frc5687.rapidreact.subsystems;

import java.sql.ClientInfoStatus;

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
    public void setBase(){
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
     * Set the status of the shooting
     * @param status 0 - Right ball / 1 - Wrong ball / 2 - Aming / 3 - Shooting
     */
    public void setShootingStatus(int status){
        switch(status){
            case 0:
                _mode = Mode.RIGHT_BALL;
                break;
            case 1:
                _mode = Mode.WRONG_BALL;
                break;
            case 2:
                _mode = Mode.AIMING;
                break;
            case 3:
                _mode = Mode.SHOOTING;
                break;
        }
    }

    public void setIntakeStatus(){
        _mode = Mode.INTAKE;
    }

    public void setClimbingStatus(){
        _mode = Mode.CLIMBING;
    }

    /**
     * Decide what color to set the blinkens to
     */
    @Override
    public void periodic(){
        switch(getMode()){
            case UNKNOW:
                break;
            case BASE:
                set(Constants.Lights.BASE);
                break;
            case WRONG_BALL:
                set(Constants.Lights.WRONG_BALL);
                break;
            case RIGHT_BALL:
                set(Constants.Lights.RIGHT_BALL);
                break;
            case SHOOTING:
                set(Constants.Lights.SHOOTING);
                break;
            case INTAKE:
                set(Constants.Lights.INTAKE);
                break;
            case CLIMBING:
                set(Constants.Lights.CLIMBING);
                break;
            case AIMING:
                set(Constants.Lights.AIMING);
                break;
        }
    }

    @Override
    public void updateDashboard() {
        metric("Blikens done", _running);
        metric("Blinken mode", _mode.toString());
    }

    /**
     * The modes of the blinkens
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
