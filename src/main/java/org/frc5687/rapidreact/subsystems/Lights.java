package org.frc5687.rapidreact.subsystems;

import java.sql.ClientInfoStatus;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends OutliersSubsystem{

    private Spark _blinkens;
    private boolean _running;
    
    private boolean _hasBall = false;
    private boolean _rightColor = false;
    private boolean _climbing = false;
    private boolean _intakeRunning = false;
    private boolean _onTarget = false;
    private boolean _aiming = false;
    private boolean _shooting = false;
    
    public Lights(OutliersContainer container, int port) {
        super(container);
        _blinkens = new Spark(port);
        _running = true;
    }

    /**
     * Sets the color of the blinkens.
     * Marking this as private to avoid confusion, since we don't want other subsystems calling it directly.
     * @param color
     */
    private void set(double color){
        _blinkens.set(color);
    }

    public void setHasBall(boolean value){
        _hasBall = value;
    }

    public void setRightColor(boolean value){
        _rightColor = value;
    }

    public void setIntakeRunning(boolean value){
        _intakeRunning = value;
    }

    public void setClimbing(boolean value){
        _climbing = value;
    }

    public void setOnTarget(boolean value){
        _onTarget = value;
    }

    public void setAiming(boolean value){
        _aiming = value;
    }

    public void setShooting(boolean value){
        _shooting = value;
    }

    private double getColor(){
        if(_climbing){
            return Constants.Lights.CLIMBING;
        }
        if(_hasBall && _rightColor){
            return _onTarget ? Constants.Lights.RIGHT_BALL_ON_TARGET: Constants.Lights.RIGHT_BALL;
        }
        if(_hasBall && !_rightColor){
            return _onTarget ? Constants.Lights.WRONG_BALL_ON_TARGET: Constants.Lights.WRONG_BALL;
        }
        if(_intakeRunning){
            return Constants.Lights.INTAKE;
        }
        if(_aiming){
            return Constants.Lights.AIMING;
        }
        if(_shooting){
            return Constants.Lights.SHOOTING;
        }

        return Constants.Lights.BASE;
    }

    /**
     * Decide what color to set the blinkens to
     */
    @Override
    public void periodic(){
        set(getColor());
        }

    @Override
    public void updateDashboard() {
        metric("Blikens done", _running);
    }
}
