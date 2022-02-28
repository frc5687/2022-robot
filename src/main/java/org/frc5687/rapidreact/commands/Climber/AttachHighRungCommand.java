package org.frc5687.rapidreact.commands.Climber;

import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Climber;

/**
 * Second step in climb.  We simply fully retract the stationary arm.
 */
public class AttachHighRungCommand extends OutliersCommand{

    private Climber _climber;
    private Step _step = Step.START;
    private long _waitUntil;
    
    public AttachHighRungCommand(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
        info("Instantiated AttachHighRungCommand");
    }

    @Override
    public void initialize(){
        super.initialize();
        info("Initialized AttachHighRungCommand");
        _step = Step.START;
    }

    @Override
    public void execute(){
        super.execute();
        switch(_step) {
            case START:
                _step = Step.ROCKOUT;
                info("AttachHighRungCommand advancing to ROCKOUT step.");
                break;
            case ROCKOUT:
                _climber.rockerOut();
                _step = Step.WAIT_ROCK;
                info("AttachHighRungCommand advancing to WAIT_ROCK step.");
                _waitUntil = System.currentTimeMillis() + Constants.Climber.ROCKER_PISTON_WAIT;
                break;
            case WAIT_ROCK:
                if ( System.currentTimeMillis() > _waitUntil) {
                    _step = Step.RETRACT_ROCKER;
                    info("AttachHighRungCommand advancing to RETRACT_ROCKER step.");
                }
                break;
            case RETRACT_ROCKER:
                _climber.setRockGoalMeters(Constants.Climber.ROCKER_MID_METERS);
                _step = Step.WAIT_ROCKER;
                info("AttachHighRungCommand advancing to WAIT_ROCKER step.");
                break;
            case WAIT_ROCKER:
                if (_climber.isRockAtGoal()) {
                    _step = Step.EXTEND_STATIONARY;
                    info("AttachHighRungCommand advancing to EXTEND_STATIONARY step.");
                }
                break;
            case EXTEND_STATIONARY:
                _climber.setStaGoalMeters(Constants.Climber.STATIONARY_CLOSE_METERS);
                _step = Step.WAIT_STATIONARY;
                info("AttachHighRungCommand advancing to WAIT_STATIONARY step.");
                break;
            case WAIT_STATIONARY:
                break;
        }

        _climber.runControllers();
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        if (_step == Step.WAIT_STATIONARY && _climber.isStaAtGoal()) {
            info("Finished AttachHighRungCommand.");
            return true;
        }; 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stop();
        error("Ended AttachHighRungCommand.");
    }

    public enum Step{
        START(0),
        ROCKOUT(1),
        WAIT_ROCK(2),
        RETRACT_ROCKER(3),
        WAIT_ROCKER(4),
        EXTEND_STATIONARY(5),
        WAIT_STATIONARY(6);

        private final int _value;
        Step(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}
