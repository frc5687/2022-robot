package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.Catapult;

public class LowerCatapult extends OutliersCommand {

    private final Catapult _catapult;

    public LowerCatapult(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
        _catapult.setWinchPosition(Constants.Catapult.WINCH_BOTTOM_LIMIT);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _catapult.isArmLowered();
    }
}
