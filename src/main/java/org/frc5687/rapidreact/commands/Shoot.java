package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class Shoot extends OutliersCommand {
    private Catapult _catapult;

    public Shoot(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        _catapult.releaseArm();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
