package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class Reset extends OutliersCommand {
    private Catapult _catapult;

    public Reset(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _catapult.setSpringMotorSpeed(-0.5);
    }

    @Override
    public boolean isFinished() {
        if (_catapult.isSpringHallTriggered()) {
            _catapult.releaseArm();
            return true;
        }
        return false;
    }
}

