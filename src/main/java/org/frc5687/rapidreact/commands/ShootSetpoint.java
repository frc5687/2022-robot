package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class ShootSetpoint extends OutliersCommand {

    private Catapult _catapult;
    private double _position;

    public ShootSetpoint(Catapult catapult, double position) {
        _catapult = catapult;
        _position = position;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        _catapult.setSpringPosition(_position);
    }

    @Override
    public boolean isFinished() {
        return _catapult.isSpringAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
