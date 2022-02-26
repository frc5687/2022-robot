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
    private long _wait;
    
    public AttachHighRungCommand(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize(){
        super.initialize();
        _step = Step.START;
    }

    @Override
    public void execute(){
        super.execute();
        switch(_step) {
            case START:
                _step = Step.ROCKOUT;
                break;
            case ROCKOUT:
                _climber.rockerOut();
                _step = Step.WAIT_ROCK;
                _wait = System.currentTimeMillis() + Constants.Climber.ROCKER_PISTON_WAIT;
                break;
            case WAIT_ROCK:
                if ( System.currentTimeMillis() > _wait) {
                    _step = Step.RETRACT_ROCKER;
                }
                break;
            case RETRACT_ROCKER:
                _climber.setRockGoal(Constants.Climber.ROCKER_MID_POSITION);
                _step = Step.WAIT_ROCKER;
                break;
            case WAIT_ROCKER:
                if (_climber.isRockAtGoal()) {
                    _step = Step.EXTEND_STATIONARY;
                }
                break;
            case EXTEND_STATIONARY:
                _climber.setStaGoal(Constants.Climber.STATIONARY_CLOSE_POSITION);
                _step = Step.WAIT_STATIONARY;
                break;
            case WAIT_STATIONARY:
                break;
        }

        _climber.runControllers();
    }


    @Override
    public boolean isFinished(){
        super.isFinished();
        return _step == Step.WAIT_STATIONARY && _climber.isStaAtGoal(); 
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _climber.stopStationaryArm();
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
