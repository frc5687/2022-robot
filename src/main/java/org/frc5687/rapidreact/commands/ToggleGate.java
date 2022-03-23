package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.util.ServoStop;

public class ToggleGate extends OutliersCommand {
    private Catapult _catapult;
    private ServoStop servoStop;

    public ToggleGate(Catapult catapult){
        _catapult = catapult;
    }

    @Override
    public void initialize(){
      super.initialize();
    }

    public void indexerToggle() {
        if (_catapult.getGateState()){
            _catapult.lowerGate();
        }
        else if (!_catapult.getGateState()){
            _catapult.raiseGate();
        }
    }

}