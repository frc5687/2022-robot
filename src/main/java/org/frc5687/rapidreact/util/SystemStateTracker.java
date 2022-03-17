package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.Notifier;

public class SystemStateTracker extends OutliersProxy {
    private static final double DEFAULT_PERIOD = 0.05; // Seconds
    private static final int DEFAULT_HISTORY = 1; // Seconds
    private static final long EPOCH = 1552693300000L;
    int _nextwrite = 0;
    private boolean _async;
    private double _period;
    private int _history;
    private int _bufferSize;
    private ISystemTrackable _trackable;
    private SystemState[] _states;


    public SystemStateTracker(ISystemTrackable trackable, boolean async, double period, int history) {
        metric("Mode/Async", async);
        metric("Mode/Period", period);
        metric("Mode/History", history);
        _trackable = trackable;
        _async = async;
        _period = period;
        _history = history;
        _bufferSize = (int)(_history * _period * 1000);
        _states = new SystemState[_bufferSize];

        if (async) {
            new Notifier(this::collect).startPeriodic(_period);
        }
    }

    public SystemStateTracker(ISystemTrackable trackable, double period, int history) {
        this(trackable, true, period, history);
    }

    public SystemStateTracker(ISystemTrackable trackable) {
        this(trackable, true, DEFAULT_PERIOD, DEFAULT_HISTORY);
    }

    public SystemStateTracker(double period, int history) {
        this(null, false, period, history);
    }

    public SystemStateTracker() {
        this(null, false, DEFAULT_PERIOD, DEFAULT_HISTORY);
    }

    synchronized public void reset() {
        _nextwrite = 0;
        _states = new SystemState[_bufferSize];
    }

    public void collect(SystemState state) {
        if (_async) {
            throw new RuntimeException("Client code must not call collect on an asynchronous PoseTracker.");
        }
        add(state);
    }

    synchronized private void collect() {
        if (_async && _trackable == null) {
            throw new RuntimeException("Asynchronous collect called on a PoseTracker instance with no trackable.");
        }
        SystemState state = _trackable.getState();
        metric("PoseCollected", state.getMillis() - EPOCH);
        add(state);
    }


    synchronized public void add(SystemState state) {
        _states[_nextwrite] = state;
        _nextwrite++;
        if (_nextwrite >= _bufferSize) {
            _nextwrite = 0;
        }
    }

    synchronized public SystemState get(long millis) {
        SystemState afterState = null;
        int read = _nextwrite - 1;
        while (true) {
            if (read < 0) {
                read = _bufferSize - 1;
            }
            // If there's nothing there, we're out of buffer
            if (_states[read] == null) {
                metric("StateFound", afterState==null?0:afterState.getMillis() - EPOCH );
                return afterState;
            }

            // If the pose time is before the requested time, we'er out of buffer
            if (_states[read].getMillis() < millis) {
                metric("PoseFound", afterState==null?0:afterState.getMillis() - EPOCH );
                return afterState;
            }

            // Otherwise grab the pose
            afterState = _states[read];

            read--;
        }

    }

    synchronized public SystemState getLatestState() {
        SystemState afterState = null;
        int read = _nextwrite - 1;
        if (read < 0) {
            read = _bufferSize - 1;
        }
        // If there's nothing there, we're out of buffer
        if (_states[read] == null) {
            return null;
        } else {
            return _states[read];
        }
    }


    synchronized public void updateDashboard() {
        SystemState state = getLatestState();
        if (state !=null) {
            state.updateDashboard("PoseTracker");
        }
    }
}
