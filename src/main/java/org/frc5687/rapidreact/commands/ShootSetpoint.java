package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Catapult;

public class ShootSetpoint extends OutliersCommand {

    private Catapult _catapult;
    private double _position;
    private double _angle;

    public ShootSetpoint(Catapult catapult, double position, double angle) {
        _catapult = catapult;
        _position = position;
        _angle = angle;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
        _catapult.setSpringDistance(_position);
        _catapult.setWinchGoal(_angle);
    }

    @Override
    public void execute() {
        super.execute();
        _catapult.shootingLights();
        _catapult.setWinchMotorSpeed(_catapult.getWinchControllerOutput());
    }

    @Override
    public boolean isFinished() {
        return _catapult.isSpringAtPosition() && _catapult.isWinchAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _catapult.setBase();
    }
}
