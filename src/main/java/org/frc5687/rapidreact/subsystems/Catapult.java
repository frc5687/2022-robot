package org.frc5687.rapidreact.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    public enum CatapultState {
        // Robot starts in ZEROING state, assuming the following:
        // - no tension on spring (should trigger spring Hall effect)
        // - catapult arm lowered
        // - pin locked
        // If spring Hall effect is triggered, robot enters LOWERING_ARM 
        ZEROING(0),

        // LOWERING_ARM checks if arm Hall effect is triggered.
        // ** DANGER ** winching too far will break robot
        // Winches down until arm Hall effect triggers.
        // If arm Hall effect is triggered, robot enters LOADING
        LOWERING_ARM(1),

        // LOADING locks pin, check for which color ball we have.
        // Depending on the ball the robot enters the AIMING state or WRONG_BALL state.
        LOADING(2),

        // AIMING waited for the OI aim button and if the drivetrain is within tolerance.
        // Also have an override button if vision is not working.
        // Change state to SHOOTING.
        AIMING(3),

        // We have the wrong ball, set the Winch and Spring goal to remove the ball.
        WRONG_BALL(4),

        // Set the winch goal and spring goal.
        // SHOOTING waits for shoot button to be pressed and the goals to be in tolerance.
        // If shoot button pressed, releases pin
        // then enters LOWERING_ARM
        SHOOTING(5),

        // If intake is up lock the catapult;
        LOCK_OUT(6),

        // if in debug, set the winch and spring settings for initial position
        PRELOAD(7),
        // Until we have figured out catapult, start in DEBUG state
        // Check that everything looks good, then press
        // button to get into ZEROING state
        // Allow manual pin release and pin lock
        DEBUG(8);


        private final int _value;
        CatapultState(int value) { _value = value; }

        public int getValue() { return _value; }
    }

    private enum PinPosition {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        LOCKED(DoubleSolenoid.Value.kReverse),
        RELEASED(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;
        PinPosition(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }
    }

    /** Catapult constructor */
    public Catapult(OutliersContainer container) {
        super(container);

        // Motor controllers (Spark Maxes)

        // Spring motor
        _springMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPRING_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _springMotor.restoreFactoryDefaults();
        _springMotor.setInverted(SPRING_MOTOR_INVERTED);
        _springMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _springMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        // Winch motor
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor.restoreFactoryDefaults();
        _winchMotor.setInverted(WINCH_MOTOR_INVERTED);
        _winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        //Save changes into flash memory of spark maxes
        _springMotor.burnFlash();
        _winchMotor.burnFlash();

        // Pneumatics (catapult locking pin)
        _releasePin = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RobotMap.PCH.RELEASE_PIN_HIGH,
                RobotMap.PCH.RELEASE_PIN_LOW);

        // Hall effects (sense position of springs and catapult arm)
        _springHall = new HallEffect(RobotMap.DIO.SPRING_HALL_EFFECT);
        _armHall = new HallEffect(RobotMap.DIO.ARM_HALL_EFFECT);

        // Encoders
        _springEncoder = _springMotor.getEncoder();
        _winchEncoder = _winchMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, COUNTS_PER_REVOLUTION);

        // PID controllers
        _springController = new ProfiledPIDController(
                SPRING_kP,
                SPRING_kI,
                SPRING_kD,
                new TrapezoidProfile.Constraints(
                        MAX_SPRING_VELOCITY_MPS,
                        MAX_SPRING_ACCELERATION_MPSS
                )
        );
        _springController.setTolerance(SPRING_TOLERANCE);
        _springController.setIntegratorRange(-SPRING_IZONE, SPRING_IZONE);

        _winchController = new ProfiledPIDController(
                WINCH_kP,
                WINCH_kI,
                WINCH_kD,
                new TrapezoidProfile.Constraints(
                        MAX_WINCH_VELOCITY_MPS,
                        MAX_WINCH_ACCELERATION_MPSS
                )
        );
        _winchController.setTolerance(WINCH_TOLERANCE);

        // set state
        _springEncoderZeroed = false;
        _winchEncoderZeroed = false;
        _state = CatapultState.DEBUG;
    }


    @Override
    public void periodic() {
        super.periodic();

        if (isSpringHallTriggered()) {
            _springEncoder.setPosition(Constants.Catapult.SPRING_BOTTOM_LIMIT);
            _springEncoderZeroed = true;
        }
//
//        if (isArmLowered() && !_winchEncoderZeroed) {
//            error("Resetting winch");
//            _winchEncoder.setPosition(WINCH_BOTTOM_LIMIT); // conversion is weird
//            _winchEncoderZeroed = true;
//        }

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

    public double getWinchRotation() {
        return _winchEncoder.getPosition();
    }

    public void zeroWinchEncoder() {
        if (!_winchEncoderZeroed) {
            _winchEncoder.setPosition(WINCH_BOTTOM_LIMIT);
            _winchEncoderZeroed = true;
        }
    }
    public boolean isWinchZeroed() {
        return _winchEncoderZeroed;
    }

    public boolean isSpringZeroed() {
        return _springEncoderZeroed;
    }

    // meters
    public double getSpringRailPosition() {
        return (getSpringEncoderRotation() / GEAR_REDUCTION) * SPRING_WINCH_DRUM_CIRCUMFERENCE;
    }

    public double getWinchStringLength() {
        return getWinchRotation() * ARM_WINCH_DRUM_CIRCUMFERENCE;
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

    public double springDisplacement() {
        return getSpringRailPosition() - 0.0;
    }

    public void runSpringController() {
        setSpringMotorSpeed(
            _springController.calculate(getSpringRailPosition()) + springDisplacement() * SPRING_DISPLACEMENT_FACTOR);
    }

    public void setSpringGoal(double position) {
        _springController.setGoal(position);
    }

    public void setWinchGoal(double stringLength) {
        _winchController.setGoal(-stringLength);
    }

    public void runWinchController() {
        setWinchMotorSpeed(
            _winchController.calculate(getWinchStringLength()));
    }

    public boolean isSpringAtPosition() {
        return _springController.atGoal();
    }

    public boolean isWinchAtGoal() {
        return _winchController.atGoal();
    }

    public void lockArm() {
        _releasePin.set(PinPosition.LOCKED.getSolenoidValue());
    }

    public void releaseArm() {
        _releasePin.set(PinPosition.RELEASED.getSolenoidValue());
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
        double force = springDisplacement * SPRING_RATE;
        double torque = LEVER_ARM_LENGTH * force;
        double angularAcceleration = torque / INERTIA_OF_ARM;
        double time = Math.sqrt((2.0 / angularAcceleration) * angleDisplacement);
        double angularVelocity = angleDisplacement / time;
        return angularVelocity * ARM_LENGTH;
    }

    public boolean isReleasePinLocked() {
        return _releasePin.get() == PinPosition.LOCKED.getSolenoidValue();
    }

    public boolean isReleasePinReleased() {
        return _releasePin.get() == PinPosition.RELEASED.getSolenoidValue();
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
        // Get the release pin's position
        DoubleSolenoid.Value current = _releasePin.get();
        if (current == PinPosition.LOCKED.getSolenoidValue()) {
            // Catapult arm locked
            return PinPosition.LOCKED;
        } else if (current == PinPosition.RELEASED.getSolenoidValue()) {
            // Catapult arm released
            return PinPosition.RELEASED;
        }
        return PinPosition.UNKNOWN;
    }

    @Override
    public void updateDashboard() {
        // Spring values
//        metric("Spring encoder rotations", getSpringEncoderRotation());
        metric("Spring rail position", getSpringRailPosition());
        metric("Spring motor output", _springMotor.getAppliedOutput());
        metric("Spring goal", _springController.getGoal().position);
        metric("Spring Hall Effect", isSpringHallTriggered());

        // Winch values
        metric("Winch rotation", getWinchRotation());
        metric("Winch controller output", _winchMotor.getAppliedOutput());
        metric("winch goal", _winchController.getGoal().position);
        metric("Winch string length", getWinchStringLength());
//        metric("String length", stringLengthToAngle(getArmReleaseAngle()));
//        metric("Winch goal", Units.radiansToDegrees(stringLengthToAngle(_winchController.getGoal().position)));

        // Catapult arm values
        metric("Arm state", getState()._value);
        metric("Arm release angle", getArmReleaseAngle());
        metric("Arm Hall Effect", isArmLowered());
    }

}
