package org.frc5687.rapidreact.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

public class Catapult extends OutliersSubsystem {

    private final CANSparkMax _springMotor;
    private final CANSparkMax _winchMotor;

    private final RelativeEncoder _springEncoder;
    private final RelativeEncoder _winchEncoder;

    private final ProfiledPIDController _springController;
    private final ProfiledPIDController _winchController;

    private final DoubleSolenoid _releasePin;

    public Catapult(OutliersContainer container) {
        super(container);

        // Create Motor controllers.
        _springMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPRING_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);

        // create Pneumatics stuff
        _releasePin = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RobotMap.PCH.RELEASE_PIN_HIGH,
                RobotMap.PCH.RELEASE_PIN_LOW);

        // setup controllers
        _springMotor.restoreFactoryDefaults();
        _winchMotor.restoreFactoryDefaults();
        _springMotor.setInverted(Constants.Catapult.SPRING_MOTOR_INVERTED);
        _winchMotor.setInverted(Constants.Catapult.WINCH_MOTOR_INVERTED);

        // setup encoder.
        _springEncoder = _springMotor.getEncoder();
        _winchEncoder = _winchMotor.getEncoder();

        // setup controllers
        _springController = new ProfiledPIDController(
                Constants.Catapult.SPRING_kP,
                Constants.Catapult.SPRING_kI,
                Constants.Catapult.SPRING_kD,
                new TrapezoidProfile.Constraints(
                        Constants.Catapult.MAX_SPRING_VELOCITY,
                        Constants.Catapult.MAX_SPRING_ACCELERATION
                )
        );

        _winchController = new ProfiledPIDController(
                Constants.Catapult.WINCH_kP,
                Constants.Catapult.WINCH_kI,
                Constants.Catapult.WINCH_kD,
                new TrapezoidProfile.Constraints(
                        Constants.Catapult.MAX_WINCH_VELOCITY,
                        Constants.Catapult.MAX_WINCH_ACCELERATION
                )
        );

        _springController.setTolerance(Constants.Catapult.SPRING_TOLERANCE);
        _winchController.setTolerance(Constants.Catapult.WINCH_TOLERANCE);
    }


    @Override
    public void periodic() {
        super.periodic();
    }

    public void setSpringMotorSpeed(double speed) {
        _springMotor.set(speed);
    }

    public void setWinchMotorSpeed(double speed) {
        _winchMotor.set(speed);
    }

    public double getSpringEncoderRotation() {
        return _springEncoder.getPosition();
    }

    public double getWinchEncoderRotation() {
        return _winchEncoder.getPosition();
    }

    // meters
    public double getSpringRailPosition() {
        return (getSpringEncoderRotation() / Constants.Catapult.GEAR_REDUCTION)
                * Constants.Catapult.ROTATIONS_TO_POSITION;
    }

    public double getWinchRotation() {
        return getWinchEncoderRotation() / Constants.Catapult.GEAR_REDUCTION;
    }

    public void setSpringPosition(double position) {
        setSpringMotorSpeed(_springController.calculate(getSpringRailPosition(), position));
    }

    public void setWinchPosition(double rotation) {
        setWinchMotorSpeed(_winchController.calculate(getWinchRotation(), rotation));
    }

    public boolean isSpringAtPosition() {
        return _springController.atGoal();
    }

    public boolean isWinchAtRotation() {
        return _winchController.atGoal();
    }

    public void releasePinOut() {
        _releasePin.set(PinPosition.OUT.getSolenoidValue());
    }

    public void releasePinIn() {
        _releasePin.set(PinPosition.IN.getSolenoidValue());
    }

    public boolean isReleasePinOut() {
        return _releasePin.get() == PinPosition.OUT.getSolenoidValue();
    }

    public boolean isReleasePinIn() {
        return _releasePin.get() == PinPosition.IN.getSolenoidValue();
    }



    public PinPosition getPinPosition() {
        //Get the release pins position
        DoubleSolenoid.Value current = _releasePin.get();
        if (current == PinPosition.OUT.getSolenoidValue()) {
            return PinPosition.OUT;
        } else if (current == PinPosition.IN.getSolenoidValue()) {
            return PinPosition.IN;
        }
        return PinPosition.UNKNOWN;
    }

    @Override
    public void updateDashboard() {
        metric("Spring Encoder Rail Rotations", getSpringRailPosition());
    }

    private enum PinPosition {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        OUT(DoubleSolenoid.Value.kReverse),
        IN(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        PinPosition(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }
    }
}
