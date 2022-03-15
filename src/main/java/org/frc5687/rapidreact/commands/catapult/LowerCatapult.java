package org.frc5687.rapidreact.commands.catapult;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;

public class LowerCatapult extends OutliersCommand {

    private final Catapult _catapult;
    private final boolean _zeroing;

    public LowerCatapult(Catapult catapult, boolean zeroing) {
        _catapult = catapult;
        _zeroing = zeroing;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
//        _catapult.setWinchGoal(Constants.Catapult.WINCH_BOTTOM_LIMIT);
    }

    @Override
    public void execute() {
        super.execute();
        if (_zeroing) {
            _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
            _catapult.setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
            if (_catapult.isArmLowered()) {
                _catapult.setWinchMotorSpeed(0);
            }
            if (_catapult.isSpringHallTriggered()) {
                _catapult.setSpringMotorSpeed(0);
            }
        } else {
            _catapult.setWinchMotorSpeed(Constants.Catapult.LOWERING_SPEED);
        }
//        _catapult.setSpringMotorSpeed(-0.1);
//        _catapult.runWinchController();
    }

    @Override
    public boolean isFinished() {
        if (_zeroing) {
            if (_catapult.isSpringHallTriggered() && _catapult.isArmLowered()) {
                _catapult.zeroWinchEncoder();
                _catapult.setWinchMotorSpeed(0);
                _catapult.setSpringMotorSpeed(0);
                _catapult.zeroSpringEncoder();
                _catapult.lockArm();// latch ;
                return true;
            }
        }
        if (_catapult.isArmLowered() && !_zeroing) {
            error("exiting command");
            _catapult.setWinchMotorSpeed(0);
            _catapult.lockArm();// latch ;
            return true;
        }
        return false;
    }
}
