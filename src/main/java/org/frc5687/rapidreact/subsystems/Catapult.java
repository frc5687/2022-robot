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
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;

import static org.frc5687.rapidreact.Constants.Catapult.*;

public class Catapult extends OutliersSubsystem {

    private final CANSparkMax _springMotor;
    private final CANSparkMax _winchMotor;

    private final RelativeEncoder _springEncoder;
    private final RelativeEncoder _winchEncoder;

    private final ProfiledPIDController _springController;
    private final ProfiledPIDController _winchController;

    private final DoubleSolenoid _releasePin;

    private final HallEffect _springHall;
    private final HallEffect _armHall;

    private boolean _springEncoderZeroed = false;
    private boolean _winchEncoderZeroed = false;

    private CatapultState _state;

    public Catapult(OutliersContainer container) {
        super(container);

        // Create Motor controllers.
        _springMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPRING_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);

        // create Pneumatics stuff
        _releasePin = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RobotMap.PCH.RELEASE_PIN_HIGH,
                RobotMap.PCH.RELEASE_PIN_LOW);

        // create hall effects.
        _springHall = new HallEffect(RobotMap.DIO.SPRING_HALL_EFFECT);
        _armHall = new HallEffect(RobotMap.DIO.ARM_HALL_EFFECT);


        // setup controllers
        _springMotor.restoreFactoryDefaults();
        _winchMotor.restoreFactoryDefaults();
        _springMotor.setInverted(SPRING_MOTOR_INVERTED);
        _winchMotor.setInverted(WINCH_MOTOR_INVERTED);
        _springMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        // setup encoder.
        _springEncoder = _springMotor.getEncoder();
        _winchEncoder = _winchMotor.getEncoder();

        //Save changes into flash memory of spark maxes
        _winchMotor.burnFlash();
        _springMotor.burnFlash();

        // setup controllers
        _springController = new ProfiledPIDController(
                SPRING_kP,
                SPRING_kI,
                SPRING_kD,
                new TrapezoidProfile.Constraints(
                        MAX_SPRING_VELOCITY_MPS,
                        MAX_SPRING_ACCELERATION_MPSS
                )
        );

        _winchController = new ProfiledPIDController(
                WINCH_kP,
                WINCH_kI,
                WINCH_kD,
                new TrapezoidProfile.Constraints(
                        MAX_WINCH_VELOCITY_MPS,
                        MAX_WINCH_ACCELERATION_MPSS
                )
        );

        _springController.setTolerance(SPRING_TOLERANCE);
        _winchController.setTolerance(Constants.Catapult.WINCH_TOLERANCE);
        _springEncoderZeroed = false;
        _winchEncoderZeroed = false;
        _state = CatapultState.IDLE;
    }


    @Override
    public void periodic() {
        super.periodic();
        if (isSpringHallTriggered()) {
//            error("Resetting Spring");
            _springEncoder.setPosition(Constants.Catapult.SPRING_BOTTOM_LIMIT);
            _springEncoderZeroed = true;
            _state = CatapultState.LOADING;
        } else if(!isSpringHallTriggered() && _springEncoderZeroed) {
            _springEncoderZeroed = false;
        }

        if (isArmLowered() && !_winchEncoderZeroed) {
            error("Resetting winch");
            _winchEncoder.setPosition(WINCH_BOTTOM_LIMIT); // conversion is weird
            _winchEncoderZeroed = true;
        } else if (!isArmLowered() && _winchEncoderZeroed) {
            _winchEncoderZeroed = false;
        }

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
        return (getSpringEncoderRotation() / GEAR_REDUCTION) * SPRING_WINCH_DRUM_CIRCUMFERENCE;
    }

    public double getWinchRotation() {
        return getWinchEncoderRotation() / GEAR_REDUCTION;
    }

    // currently, angle is STOWED_ANGLE all the way down not referencing a plane.
    public double getArmReleaseAngle() {
        return STOWED_ANGLE - stringLengthToAngle(getWinchRotation() * ARM_WINCH_DRUM_CIRCUMFERENCE);
    }

    //meters and radians.
    protected double stringLengthToAngle(double stringLength) {
        return LINEAR_REGRESSION_SLOPE * stringLength + LINEAR_REGRESSION_OFFSET;
    }
    // radians
    protected double angleToStringLength(double angle) {
        return (angle - STOWED_ANGLE + LINEAR_REGRESSION_OFFSET) / LINEAR_REGRESSION_SLOPE;
    }

    public void runSpringController() {
        setSpringMotorSpeed(_springController.calculate(getSpringEncoderRotation()));
    }

    public void setSpringGoal(double position) {
        _springController.setGoal(position);
    }

    public void setWinchGoal(double angle) {
        _winchController.setGoal(angleToStringLength(angle));
    }

    public void runWinchController() {
        setWinchMotorSpeed(_winchController.calculate(getWinchRotation()));
    }

    public boolean isSpringAtPosition() {
        return _springController.atGoal();
    }

    public boolean isWinchAtGoal() {
        return _winchController.atGoal();
    }

    public void lockArm() {
        _releasePin.set(PinPosition.OUT.getSolenoidValue());
    }

    public void releaseArm() {
        _releasePin.set(PinPosition.IN.getSolenoidValue());
    }

    // calculate the spring displacement based on angle displacement.
    public double calculateSpringDisplacement(double angleDisplacement, double velocity) {
        return ((velocity * velocity) * 2.0 * INERTIA_OF_ARM) /
                ((ARM_LENGTH * ARM_LENGTH) * angleDisplacement * SPRING_RATE * LEVER_ARM_LENGTH);
    }

    // calculated the ball exit velocity in meters per sec from the angle displacement.
    public double calculateExitVelocity(double angle, double spring) {
        double springDisplacement = Math.abs(spring - SPRING_BOTTOM_LIMIT);
        double angleDisplacement = Math.abs(angle - STOWED_ANGLE);
        double force = springDisplacement * Constants.Catapult.SPRING_RATE;
        double torque = Constants.Catapult.LEVER_ARM_LENGTH * force;
        double angularAcceleration = torque / Constants.Catapult.INERTIA_OF_ARM;
        double time = Math.sqrt((2.0 / angularAcceleration) * angleDisplacement);
        double angularVelocity = angleDisplacement / time;
        return angularVelocity * Constants.Catapult.ARM_LENGTH;
    }

    public boolean isReleasePinOut() {
        return _releasePin.get() == PinPosition.OUT.getSolenoidValue();
    }

    public boolean isReleasePinIn() {
        return _releasePin.get() == PinPosition.IN.getSolenoidValue();
    }

    public boolean isSpringHallTriggered() { return _springHall.get(); }
    public boolean isArmLowered() { return _armHall.get(); }

    public CatapultState getState() {
        return _state;
    }

    public void setState(CatapultState state) {
        _state = state;
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
        metric("Spring encoder rotations", getSpringEncoderRotation());
        metric("Spring rail position", getSpringRailPosition());
        metric("Spring motor output", _springMotor.getAppliedOutput());
        metric("Spring goal", _springController.getGoal().position);

        metric("Winch rotation", getWinchRotation());
        metric("Arm release angle", getArmReleaseAngle());
        metric("Winch controller output", _winchMotor.getAppliedOutput());
        metric("Winch goal", _winchController.getGoal().toString());

        metric("Arm Hall Effect", isArmLowered());
        metric("Spring Hall Effect", isSpringHallTriggered());
    }

    public enum CatapultState {
        IDLE(0),
        LOWERING_ARM(1),
        LOADING(2),
        AIMING(3),
        SHOOTING(4);

        private final int _value;
        CatapultState(int value) { _value = value; }
        public int getValue() { return _value; }
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
