package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.VecBuilder;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.Catapult;

public class ShootSetpoint extends OutliersCommand {
    private Catapult _catapult;

    private double _angle;
    private double _velocity;

    public ShootSetpoint(Catapult catapult, double angle, double velocity) {
        _catapult = catapult;
        _angle = angle;
        _velocity = velocity;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
        _catapult.setReference(VecBuilder.fill(_angle, _velocity / Constants.Catapult.ARM_LENGTH));
    }

    @Override
    public void execute() {
        _catapult.heuristicControl(VecBuilder.fill(_angle, _velocity / Constants.Catapult.ARM_LENGTH));
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
