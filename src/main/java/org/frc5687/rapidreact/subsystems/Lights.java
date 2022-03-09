package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends OutliersSubsystem{

    private Spark _blinkens;
    private boolean _running;
    
    public Lights(OutliersContainer container, int port) {
        super(container);
        _blinkens = new Spark(port);
        _running = true;
    }

    public void base(){
        _blinkens.set(Constants.Blinkens.PULSING_RED);
    }

    public void wrongBall(){
        _blinkens.set(Constants.Blinkens.SOLID_HOT_PINK);
    }

    public void rightBall(){
        _blinkens.set(Constants.Blinkens.BEATING_GREEN);
    }

    public void shooting(){
        _blinkens.set(Constants.Blinkens.BEATING_ORANGE);
    }

    public void intake(){
        _blinkens.set(Constants.Blinkens.RAINBOW);
    }

    @Override
    public void updateDashboard() {
        metric("Blikens done", _running);
    }
}
