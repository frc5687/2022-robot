package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends OutliersSubsystem{

    private Spark _blinkens;
    private boolean _running;
    
    public Lights(OutliersContainer container, int port) {
        super(container);
        _blinkens = new Spark(8);
        _running = true;
        _blinkens.set(0.87);
    }

    @Override
    public void updateDashboard() {
        metric("Blikens done", _running);
    }
}
