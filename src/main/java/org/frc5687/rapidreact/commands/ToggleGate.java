package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class ToggleGate extends OutliersCommand {
    private Catapult _catapult;
    private boolean _done = false;

    public ToggleGate(Catapult catapult){
        _catapult = catapult;
    }

    @Override
    public void initialize(){
      super.initialize();

        if (_catapult.getGateState()){ // If indexer is up, moves it down
                _catapult.lowerGate();
                _done = true;
        }

        else if (!_catapult.getGateState()){ // If indexer is down, moves it up
                _catapult.raiseGate();
                _done = true;
        }
    }

    @Override
    public boolean isFinished(){
        if (_done && _catapult.getGateState()){ // If indexer has been moved up, returns true
            info("Gate has been raised");
            return true;
        }
        else if (_done && !_catapult.getGateState()) { // If indexer has been moved down, returns true
            info("Gate has been lowered");
            return true;
        }
            return false; // If indexer is not at either position, or has not been moved
    }

}