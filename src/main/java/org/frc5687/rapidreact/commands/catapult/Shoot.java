package org.frc5687.rapidreact.commands.catapult;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Catapult.CatapultState;

public class Shoot extends OutliersCommand {
    private Catapult _catapult;
    private Indexer _indexer;

    private boolean _done = false;

    public Shoot(Catapult catapult, Indexer indexer) {
        _catapult = catapult;
        _indexer = indexer;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        if (!_done && _catapult.isInitialized() && _indexer.isBallDetected()) {
            info("Shooting");
            _catapult.setState(CatapultState.AIMING);
            _catapult.setAutomatShoot(true);
            _done = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (_done && (_catapult.getState() == Catapult.CatapultState.LOADING)) {
            info("Shoot finished");
            return true;
        }
        info("Shoot waiting for winch which is at " + _catapult.getWinchStringLength() );
        return false;
    }
}
