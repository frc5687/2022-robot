package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

/**
 * Second step in climb.  We simply fully retract the stationary arm.
 */
public class AttachTraversalRungCommand extends OutliersCommand{

    private Climber _climber;
    private Step _step = Step.START;
    private long _wait;
    
    public AttachTraversalRungCommand(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated AttachTraversalRungCommand");
    }

    @Override
    public void initialize(){
        super.initialize();
        _step = Step.START;
        info("Initialized AttachTraversalRungCommand");
    }

    @Override
    public void execute(){
        super.execute();
        switch(_step) {
            case START:
                _step = Step.EXTEND_STATIONARY;
                info("AttachTraversalRungCommand advancing to EXTEND_STATIONARY step.");
                break;
            case EXTEND_STATIONARY:
                _climber.setStaGoalMeters(Constants.Climber.STATIONARY_EXTENDED_METERS);
                _step = Step.WAIT_STATIONARY;
                info("AttachTraversalRungCommand advancing to WAIT_STATIONARY step.");
                break;
            case WAIT_STATIONARY:
                if (_climber.isStaAtGoal()) {
                    _climber.stopStationaryArm();
                    _step = Step.RETRACT_ROCKER;
                    info("AttachTraversalRungCommand advancing to RETRACT_ROCKER step.");
                }
                break;
            case RETRACT_ROCKER:
//                _climber.setRockGoalMeters(Constants.Climber.ROCKER_RETRACTED_METERS);
                _climber.setRockSpeed(-1.0);
                _step = Step.WAIT_ROCKER;
                info("AttachTraversalRungCommand advancing to WAIT_ROCKER step.");
                break;
            case WAIT_ROCKER:
                _climber.setRockSpeed(-1.0);
                if (_climber.isRockArmDown()) {
//                    _climber.setRockSpeed(0);
                    _climber.stopRockerArm();
                    _step = Step.ROCKIN;
                    info("AttachTraversalRungCommand advancing to ROCKIN step.");
                }
                break;
            case ROCKIN:
                _climber.rockerIn();
                _step = Step.WAIT_ROCK;
                info("AttachTraversalRungCommand advancing to WAIT_ROCK step.");
                _wait = System.currentTimeMillis() + Constants.Climber.ROCKER_PISTON_WAIT;
                break;
            case WAIT_ROCK:
                if ( System.currentTimeMillis() > _wait) {
                    _step = Step.EXTEND_ROCKER;
                    info("AttachTraversalRungCommand advancing to EXTEND_ROCKER step.");
                }
                break;
            case EXTEND_ROCKER:
                _climber.setRockGoalMeters(Constants.Climber.ROCKER_MID_METERS);
                _step = Step.WAIT_FINISH;
                info("AttachTraversalRungCommand advancing to WAIT_FINISH step.");
                break;
            case WAIT_FINISH:
                break;
        }
//        _climber.runControllers();
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        
        if (_step == Step.WAIT_FINISH && _climber.isRockAtGoal()) {
            info("Finished AttachTraversalRungCommand.");
            return true;
        }; 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stop();
        error("Ended AttachTraversalRungCommand.");
    }

    public enum Step{
        START(0),
        EXTEND_STATIONARY(1),
        WAIT_STATIONARY(2),
        RETRACT_ROCKER(3),
        WAIT_ROCKER(4),
        ROCKIN(5),
        WAIT_ROCK(6),
        EXTEND_ROCKER(7),
        WAIT_FINISH(8);

        private final int _value;
        Step(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}
