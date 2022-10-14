package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Slingshot;

public class PrimeSlingshot extends OutliersCommand {
    
    private Slingshot _slingshot;
    private SlingshotState _state;

    public PrimeSlingshot(Slingshot slingshot) {
        _slingshot = slingshot;
    }
    
    @Override
    public void initialize() {
        _state = SlingshotState.PRIMING;

    }
    
    @Override
    public void execute() {
        switch(_state) {
            case PRIMING:
                _slingshot.runTensioner();
                if (_slingshot.isPrimed()) {
                    _state = SlingshotState.PRIMED;
                }
                break;
            case PRIMED:
                _slingshot.stopTensioner();
                break;
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    
    public enum SlingshotState {
        PRIMING(0),

        PRIMED(1);

        private final int _value;
        SlingshotState(int value) { _value = value; }

        public int getValue() { return _value; }
    }

}
