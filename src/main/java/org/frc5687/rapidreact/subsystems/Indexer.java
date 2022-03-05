package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.util.OutliersContainer;

/** Keep balls separated so can intake two, shoot just one.
 * 
 * <p>Note: 2022-robot repo calls this subsystem "ServoStop"
 */
public class Indexer extends OutliersSubsystem {
    
    public enum IndexerState {
        DEPLOYED, // stop balls
        RETRACTED // allow balls to roll down
    }
    private IndexerState _state;

    /** Create an Indexer subsystem */
    public Indexer(OutliersContainer container) {
        super(container);
        _state = IndexerState.DEPLOYED;
    }

    // Query state of indexer

    public IndexerState getState() {
        return _state;
    }

    public void setState(IndexerState state) {
        _state = state;
    }

    public void retract() {
        // Retract the blocking arm
        // Let balls enter the catapult
        _state = IndexerState.RETRACTED;
    }

    public void deploy() {
        // Deploy the blocking arm
        // Stop balls from entering the catapult
        _state = IndexerState.DEPLOYED;
    }

    @Override
    public void updateDashboard() {
        // Indexer state
        metric("Indexer State", getState().name());
    }

}
