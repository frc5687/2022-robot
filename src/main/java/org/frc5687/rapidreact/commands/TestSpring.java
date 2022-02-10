package org.frc5687.rapidreact.commands;


import org.frc5687.rapidreact.subsystems.Catapult;

public class TestSpring extends OutliersCommand{

    private Catapult _catapult;

    public TestSpring(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        _catapult.setWinchGoal(-3.0);
        _catapult.setSpringGoal(100);
    }

    @Override
    public void execute() {
        super.execute();
        _catapult.runWinchController();
        _catapult.runSpringController();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(-3.0 - _catapult.getWinchRotation()) < 0.1) {
            _catapult.releasePinIn();
            return true;
        }
        return false;
    }
}
