package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class Shoot extends OutliersCommand {
    private Catapult _catapult;

    public Shoot(Catapult catapult) {
        _catapult = catapult;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        info("Start shooting");
        super.initialize();
        _catapult.setAutoshoot(true);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return _catapult.getState() == Catapult.CatapultState.LOADING || _catapult.getState() == Catapult.CatapultState.AIMING;
    }
}
