package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class Shoot extends OutliersCommand {
    private Catapult _catapult;

    private boolean _done = false;

    public Shoot(Catapult catapult) {
        _catapult = catapult;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        info("Shoot initialized");
        super.initialize();
    }

    @Override
    public void execute() {
        if (!_done && _catapult.isInitialized()) {
            info("Shooting");
            _catapult.setState(CatapultState.AIMING);
            _catapult.setAutoshoot(true);
            _done = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (_done && (_catapult.getState() == Catapult.CatapultState.LOADING || _catapult.getState() == Catapult.CatapultState.AIMING)) {
            info("Shoot finished");
            return true;
        }
        info("Shoot waiting for winch which is at " + _catapult.getWinchStringLength() );
        return false;
    }
}
