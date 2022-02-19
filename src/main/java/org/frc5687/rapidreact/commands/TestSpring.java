package org.frc5687.rapidreact.commands;


import edu.wpi.first.math.util.Units;
import org.frc5687.rapidreact.subsystems.Catapult;

public class TestSpring extends OutliersCommand{

    private Catapult _catapult;
    private double _distance;
    private double _string;

    public TestSpring(Catapult catapult, double distance, double string) {
        _catapult = catapult;
        _distance = distance;
        _string = string;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        _catapult.setWinchGoal(_string);
        _catapult.setSpringGoal(_distance); // meters
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
//            _catapult.releaseArm();
            return true;
        }
        return false;
    }
}
