package org.frc5687.rapidreact.util;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser extends OutliersProxy {
    private Position _autoPosition;
    private Mode _autoMode;

    private SendableChooser<Position> _positionChooser;
    private SendableChooser<Mode> _modeChooser;

    public AutoChooser() {
        _positionChooser = new SendableChooser<>();
        _positionChooser.setDefaultOption("First Position", Position.First);
        _positionChooser.addOption("Second Position", Position.Second);
        _positionChooser.addOption("Third Position", Position.Third);
        _positionChooser.addOption("Fourth Position", Position.Fourth);
        SmartDashboard.putData("Auto Position", _positionChooser);

        _modeChooser = new SendableChooser<>();
        _modeChooser.setDefaultOption("Zero Ball", Mode.ZeroBall);
        _modeChooser.addOption("One Ball", Mode.OneBall);
        _modeChooser.addOption("Two Ball", Mode.TwoBall);
        _modeChooser.addOption("Three Ball", Mode.ThreeBall);
        _modeChooser.addOption("Four Ball", Mode.FourBall);
        SmartDashboard.putData("Auto Mode", _modeChooser);
    }

    public void updateChooser() {
        _autoPosition = _positionChooser.getSelected();
        _autoMode = _modeChooser.getSelected();
        metric("Auto Position", _autoPosition.name());
        metric("Auto Mode", _autoMode.name());
    }

    public Position getAutoPosition() {
        return _autoPosition;
    }

    public Mode getAutoMode() {
        return _autoMode;
    }

    public void updateDashboard() {
    }

    public enum Position {
        Unknown(-1),
        First(0),
        Second(1),
        Third(2),
        Fourth(3);

        private int _value;

        Position(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
    public enum Mode {
        Unknown(-1), 
        ZeroBall(0), 
        OneBall(1),
        TwoBall(2),
        ThreeBall(3),
        FourBall(3);

        private int _value;

        Mode(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }

    }
}


