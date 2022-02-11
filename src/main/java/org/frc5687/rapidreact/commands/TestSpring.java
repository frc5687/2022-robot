package org.frc5687.rapidreact.commands;


import edu.wpi.first.math.util.Units;
import org.frc5687.rapidreact.subsystems.Catapult;

public class TestSpring extends OutliersCommand{

    private Catapult _catapult;

    public TestSpring(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        _catapult.setWinchGoal(Units.degreesToRadians(20));
        _catapult.setSpringGoal(0.2); // meters
    }

    @Override
    public void execute() {
        super.execute();
        _catapult.runWinchController();
        _catapult.runSpringController();
    }

    @Override
    public boolean isFinished() {
        if (_catapult.isWinchAtGoal()) {
            _catapult.releaseArm();
            return true;
        }
        return false;
    }
}
