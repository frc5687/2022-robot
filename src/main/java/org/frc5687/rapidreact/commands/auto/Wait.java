package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import edu.wpi.first.wpilibj.Timer;

/**
 * Wait (do nothing) for a number of seconds so we can pause during auto.
 * 
 * DEPRECATED: Use WPI's WaitCommand instead.
 * See https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
 */
public class Wait extends OutliersCommand {

    private double _waiting; // Seconds to wait
    private Timer _timer;

    /**
     * Create Wait command
     * 
     * @param delay seconds to wait
     */
    public Wait(double delay) {
        _waiting = delay;
        _timer = new Timer();
    }
   
    @Override
    public void initialize() {
        super.initialize();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return (_timer.get() >= _waiting);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
 
}
