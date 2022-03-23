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

        if (_catapult.getGateState()){ // If indexer is up, moves it down
                _catapult.lowerGate();
        }

        else if (!_catapult.getGateState()){ // If indexer is down, moves it up
                _catapult.raiseGate();
        }
    }
}