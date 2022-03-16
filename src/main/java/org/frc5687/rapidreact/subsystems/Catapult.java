package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.util.ColorSensor;
import org.frc5687.rapidreact.util.ProximitySensor;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.ServoStop;

import static org.frc5687.rapidreact.Constants.Catapult.*;

/** Catapult shoots balls with a spring-loaded arm.
 * 
 * <ul>
 *  <li>Pin solenoid latches or releases catapult arm.
 *  <li>Baby neo winds and unwinds rope to lower arm and set release angle.
 *  <li>Falcon motor tensions spring to set power.
 *  <li>Proximity sensor senses ball on arm.
 *  <li>Color sensor senses ball color.
 * </ul>
 */
public class Catapult extends OutliersSubsystem {

    // We control catapult with two motors
    private final TalonFX _springMotor; // Falcon motor controls spring tension
    private final CANSparkMax _armMotor; // Baby Neo motor controls arm position

    // Spring motor uses integrated encoder and controller
    // Arm motor uses relative encoder and PID controller
    private final RelativeEncoder _armEncoder;
    private final ProfiledPIDController _armController;

    private final DoubleSolenoid _releasePin;

    private final HallEffect _springHall;
    private final HallEffect _armHall;

    private boolean _springEncoderZeroed = false;
    private boolean _armEncoderZeroed = false;

    private CatapultState _state;
    private CatapultState _state_prior; // for ERROR and other states that need to know
    private String _state_error; // what caused ERROR state

    private ColorSensor _colorSensor;
    private ProximitySensor _proximitySensor;
    private ServoStop _gate;

    private DriverStation.Alliance _alliance;

    private double _springGoal;

    private boolean _autoShoot;

    private boolean _initialized = false;

    private CatapultSetpoint _setpoint = CatapultSetpoint.NONE;

    public enum CatapultState {

        // When first catapult command gets scheduled, catapult should be in the following physical configuration
        // ("standard starting configuration"):
        // - intake in STOWED_INITIAL state (catapult should not be fired)
        // - arm lowered
        // - pin locked
        // - spring tensioned for auto shot
        // - winch string unwound to hard stop length for auto shot
        // This allows auto to deploy intake, then release pin to shoot ball one into hub

        // Use test mode to control all catapult hardware and state manually.
        // This allows catapult to be put into standard starting configuration

        // Consider having a timer so catapult enters ERROR state if state transitions take too long

        // Consider buttons for procedures to recover from certain ERROR states

        // NEW state machine

        // When code starts, catapult state is assumed to be LOCK_OUT

        // LOCK_OUT -- intake in initial stowed position, waiting for intake to be deployed once
        LOCK_OUT(0),

        // PRELOADED -- ready to shoot first ball in auto mode, waiting to release pin
        PRELOADED(1),

        // ZEROING_ARM -- pin released, first shot taken, winch encoder being zeroed
        ZEROING_ARM(2),

        // ZEROING_SPRING -- winch encoder zeroed, spring encoder being zeroed
        ZEROING_SPRING(3),

        // LOWERING_ARM -- pin released, spring encoder zeroed, run winch to pull arm down
        LOWERING_ARM(4),

        // LATCHING -- winch hall effect triggered, zero winch encoder, turn off winch motor, latch pin
        LATCHING(5),

        // LOADING -- waiting for ball on arm (necessary to avoid dry firing)
        LOADING(6),

        // Catapult doesn't aim until it has a ball loaded.  This prevents wasting energy.

        // AIMING -- setting winch string and spring tension for next shot
        AIMING(7),

        // READY -- winch string set, spring tension set, waiting for shoot command to release pin
        READY(8),

        // From READY can go to
        // -> AIMING if things change (i.e. we drive to a different position) or to
        // -> LOWERING_ARM if shot taken

        // Additional states:

        // WAITING -- do nothing to allow catapult some cycles to transition to next state
        //  (keep track of old and new state and time remaining to wait)
        WAITING(9),

        // ERROR -- keep track of last state and reason for error
        ERROR(10),
        
        // NONE -- for ERROR (keeps track of prior state);
        NONE(11);

        // Cycles:

        // auto => LOCK_OUT -> PRELOADED -> ZEROING_ARM -> ZEROING_SPRING -> LOADING -> AIMING -> READY
        // robot moves so needs to aim again => READY -> AIMING -> READY
        // robot shoots so needs to reset catapult => READY -> LOWERING_ARM -> LATCHING -> LOADING -> AIMING -> READY
        // available when robot in test mode; do before each match => <ANY> -> AIMING -> LOCK_OUT

        // OLD state machine:

        // ZEROING assumes the following:
        // - no tension on spring (should trigger spring Hall effect)
        // - catapult arm lowered
        // - pin locked
        // If spring Hall effect is triggered, robot enters LOWERING_ARM 
        // ZEROING(0),

        // LOWERING_ARM checks if arm Hall effect is triggered.
        // ** DANGER ** winching too far will break robot
        // Winches down until arm Hall effect triggers.
        // If arm Hall effect is triggered, robot enters LOADING
        // LOWERING_ARM(1),

        // LOADING locks pin, check for which color ball we have.
        // Depending on the ball the robot enters the AIMING state or WRONG_BALL state.
        // LOADING(2),

        // AIMING waited for the OI aim button and if the drivetrain is within tolerance.
        // Also have an override button if vision is not working.
        // Change state to SHOOTING.
        // AIMING(3),

        // We have the wrong ball, set the Winch and Spring goal to remove the ball.
        // WRONG_BALL(4),

        // Set the winch goal and spring goal.
        // SHOOTING waits for shoot button to be pressed and the goals to be in tolerance.
        // If shoot button pressed, releases pin
        // then enters LOWERING_ARM
        // SHOOTING(5),

        // If intake is up lock the catapult;
        // LOCK_OUT(6),

        // if in debug, set the winch and spring settings for initial position
        // PRELOAD(7),

        // Until we have figured out catapult, start in DEBUG state
        // Check that everything looks good, then press
        // button to get into ZEROING state
        // Allow manual pin release and pin lock
        // DEBUG(8),
        // a button will stop all catapult movement, this is for the case if
        // a ball gets under the catapult.
        // KILL(9),

        // AUTO(10);

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

    /** Construct a Catapult */
    public Catapult(OutliersContainer container) {
        super(container);

        // Spring motor (Falcon)
        // See https://docs.ctre-phoenix.com/en/latest/index.html
        _springMotor = new TalonFX(RobotMap.CAN.TALONFX.CATAPULT_SPRING);
        _springMotor .setNeutralMode(NeutralMode.Brake);
//        _springMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.Climber.ARM_CURRENT_LIMIT, Constants.Climber.CURRENT_THRES, Constants.Climber.ARM_THRES_TIME));
        _springMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _springMotor.setInverted(SPRING_MOTOR_INVERTED);
        _springMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        _springMotor.configMotionAcceleration(ACCELERATION);
        _springMotor.configVoltageMeasurementFilter(8);
        _springMotor.enableVoltageCompensation(true);
        _springMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
        _springMotor.configClosedloopRamp(0,50);

        // Spring PID controller settings
        _springMotor.config_kP(MOTION_MAGIC_SLOT, SPRING_kP);
        _springMotor.config_kI(MOTION_MAGIC_SLOT, SPRING_kI);
        _springMotor.config_kD(MOTION_MAGIC_SLOT, SPRING_kD);
        _springMotor.config_kF(MOTION_MAGIC_SLOT, SPRING_kF);

        // Winch motor (Spark Max Baby Neo)
        _armMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _armMotor.restoreFactoryDefaults();
        _armMotor.setInverted(WINCH_MOTOR_INVERTED);
        _armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _armMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _armMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _armMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        _armMotor.setSmartCurrentLimit(WINCH_CURRENT_LIMIT);

        // Save changes into flash memory of spark maxes
        _armMotor.burnFlash();

        // Winch encoder (which type is it?)
        _armEncoder = _armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, COUNTS_PER_REVOLUTION);

        _armController = new ProfiledPIDController(
                WINCH_kP,
                WINCH_kI,
                WINCH_kD,
                new TrapezoidProfile.Constraints(
                        MAX_WINCH_VELOCITY_MPS,
                        MAX_WINCH_ACCELERATION_MPSS
                )
        );
        _armController.setTolerance(WINCH_TOLERANCE);

        // Hall effects (sense position of springs and catapult arm)
        _springHall = new HallEffect(RobotMap.DIO.SPRING_HALL_EFFECT);
        _armHall = new HallEffect(RobotMap.DIO.ARM_HALL_EFFECT);

        // Pneumatics (catapult locking pin)
        _releasePin = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            RobotMap.PCH.RELEASE_PIN_HIGH,
            RobotMap.PCH.RELEASE_PIN_LOW
            );

        // Indexer gate to allow 2nd onboard ball to drop onto catapult arm
        _gate = new ServoStop(RobotMap.PWM.INTAKE_STOPPER);
        // Proximity sensor (sense if ball on catapult arm)
        _proximitySensor = new ProximitySensor(RobotMap.DIO.PROXIMITY_SENSOR);
        // Color sensor (sense which color ball)
        _colorSensor = new ColorSensor(I2C.Port.kMXP);

        // set state
        _springEncoderZeroed = false;
        _armEncoderZeroed = false;
        _state = CatapultState.LOCK_OUT;
        _state_prior = CatapultState.NONE;
        _state_error = "No errors yet";
        _alliance = DriverStation.getAlliance();
        _springGoal = 0;
    }

    /** What to do with catapult once per run of the scheduler.
     * 
     * <p> Note that this periodic() method is similar to the "default command":
     * 
     * <ul>
     *   <li> Subsystems can be associated with "default commands" that will be automatically
     *   scheduled when no other command is currently using the subsystem. This is useful for
     *   continuous “background” actions such as keeping an arm held at a setpoint.
     * 
     *   <li> Similar functionality can be achieved in the subsystem’s periodic() method,
     *   which is run once per run of the scheduler; teams should try to be consistent within
     *   their codebase about which functionality is achieved through either of these methods.
     * </ul>
     * 
     */
    @Override
    public void periodic() {
        super.periodic();

        // Failsafes

        // Prevent Baby Neo winch motor from burning out
        if (isArmLowered() && (_armMotor.getAppliedOutput() > 0)) {
            setArmMotorSpeed(0);
        }
        // TODO: add a timer to allow short bursts above current limit
        if (_armMotor.getOutputCurrent() > WINCH_CURRENT_LIMIT) {
            setArmMotorSpeed(0);
        }

        // TODO: failsafe for spring motor

        // See if we need to change state
        switch(_state) {
            case LOCK_OUT:
                // Initial state of catapult
                // Intake is in initial stowed position
                // NOT safe to shoot ball
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);

                // Error conditions

                // Pin should be latched
                if (isReleasePinReleased()) {
                    _state_prior = CatapultState.LOCK_OUT;
                    _state_error = "Pin released";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Arm should be lowered
                if (!isArmLowered()) {
                    _state_prior = CatapultState.LOCK_OUT;
                    _state_error = "Arm not lowered";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Spring hall effect should not be triggered
                // In this state, spring should be under tension
                if (isSpringHallTriggered()) {
                    _state_prior = CatapultState.LOCK_OUT;
                    _state_error = "Spring hall triggered";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Success condition

                // Command will advance state when intake is deployed for first time

                // Action
                // Stay in this state until a command advances state to PRELOADED
                return;
            case PRELOADED:
                // Intake has been deployed at least once
                // Now it is safe to shoot ball
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);

                // Error conditions

                // Pin should be latched
                if (isReleasePinReleased()) {
                    _state_prior = CatapultState.LOCK_OUT;
                    _state_error = "Pin released";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Arm should be lowered
                if (!isArmLowered()) {
                    _state_prior = CatapultState.LOCK_OUT;
                    _state_error = "Arm not lowered";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Spring hall effect should not be triggered
                // In this state, spring should be under tension
                if (isSpringHallTriggered()) {
                    _state_prior = CatapultState.PRELOADED;
                    _state_error = "Spring hall triggered";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Success condition

                // Did we shoot first ball?
                if (isReleasePinReleased()) {
                    // Time to zero arm motor
                    _state = CatapultState.ZEROING_ARM;
                    return;
                }

                // Action

                // Stay in this state until arm is released
                return;
            case ZEROING_ARM:
                // First ball has been shot
                // Now lower arm to zero the arm encoder
                // Keep spring tension where it is
                // Spring motor should not be moving
                setSpringMotorSpeed(0);

                // Error conditions

                // Spring hall effect should not be triggered
                // In this state, spring should be under tension
                if (isSpringHallTriggered()) {
                    _state_prior = CatapultState.ZEROING_ARM;
                    _state_error = "Spring hall triggered";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Success condition

                // Have we already lowered arm?
                if (isArmLowered()) {
                    // Stop winch motor
                    setArmMotorSpeed(0);
                    // Try to latch release pin
                    // TODO: is it a problem to do this more than once?
                    lockArm();
                    // TODO: think more about what is going on with arm motor
                    // Momentum will carry it past the trigger point
                    // Do we want to wait to zero arm encoder?
                    // Will motor be oscillating trying to hold position?
                    // We're about where we want to be to zero encoder.
                    zeroArmEncoder();
                    // Check if we successfully latched release pin
                    // TODO: do we need to wait a bit before checking this?
                    if (isReleasePinLocked()) {
                        _state = CatapultState.ZEROING_SPRING;
                    } else {
                        // We have a problem Houston
                        _state_prior = CatapultState.ZEROING_ARM;
                        _state_error = "Release pin did not latch"; 
                        _state = CatapultState.ERROR;
                    }
                    return;
                }

                // Action

                // Lower arm
                setArmMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                return;
            case ZEROING_SPRING:
                // Arm motor has been zeroed
                // Release pin has been latched
                // Now release tension in spring to zero the spring encoder
                // Arm motor should not be moving
                setArmMotorSpeed(0);
                // Have we already released all tension in spring?
                if (isSpringHallTriggered()) {
                    // Stop spring motor
                    setSpringMotorSpeed(0);
                    // TODO: think more about what is going on with spring motor
                    // Momentum will carry it past the trigger point
                    // Do we want to wait to zero spring encoder?
                    // Will motor be oscillating trying to hold position?
                    // We're about where we want to be to zero encoder.
                    zeroSpringEncoder();
                    _state = CatapultState.LOADING;
                    return;
                }
                // Release tension in spring
                setSpringMotorSpeed(Constants.Catapult.SPRING_ZERO_SPEED);
                return;
            case LOWERING_ARM:
                // Spring motor should not be moving
                setSpringMotorSpeed(0);

                // Error conditions

                // If we're detecting a ball when the arm is not lowered, we have
                // a problem -- a ball may have sneaked in under the catapult arm
                if (!isArmLowered() && isBallDetected()) {
                    // Stop winching down arm
                    setArmMotorSpeed(0);
                    _state_prior = CatapultState.LOWERING_ARM;
                    _state_error = "Ball detected before arm lowered";
                    _state = CatapultState.ERROR;
                    return;
                }
                // Is release pin latched by mistake?
                // Release pin should not be latched before arm is lowered
                if (isReleasePinLocked()) {
                    // Stop winching down arm
                    setArmMotorSpeed(0);
                    _state_prior = CatapultState.LOWERING_ARM;
                    _state_error = "Release pin locked";
                    _state = CatapultState.ERROR;
                    return;
                }

                // Success condition

                // Check if arm lowered all the way
                if (isArmLowered()) {
                    // Stop winching down arm
                    setArmMotorSpeed(0);
                    // Try to latch release pin
                    lockArm();
                    // Set state to LATCHING
                    _state = CatapultState.LATCHING;
                    return;
                }

                // Action

                // Lower arm
                setArmMotorSpeed(Constants.Catapult.LOWERING_SPEED);
                return;
            case LATCHING:
                // Arm has been lowered, now latch release pin to hold it down
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);
                // Try to latch pin if necessary
                if (isReleasePinReleased()) {
                    lockArm();
                }
                // Set state to LOADING if release pin latched
                if (isReleasePinLocked()) {
                    _state = CatapultState.LOADING;
                    return;
                }
                return;
            case LOADING:
                // We are lowered and locked, now ready for loading
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);
                // Check to make sure arm is lowered all the way
                if (!isArmLowered()) {
                    _state_prior = CatapultState.LOADING;
                    _state_error = "Arm not lowered";
                    _state = CatapultState.ERROR;
                    return;
                }
                // Check to make sure release pin latched
                if (isReleasePinReleased()) {
                    _state_prior = CatapultState.LOADING;
                    _state_error = "Pin released";
                    _state = CatapultState.ERROR;
                    return;
                }
                // Arm is fully lowered, so might as well remember that for next time
                zeroArmEncoder();
                // We're ready to aim if we have a ball
                if (isBallDetected()) {
                    _state = CatapultState.AIMING;
                    return;
                }
                // Wait for a ball to be detected on the catapult arm
                return;
            case AIMING:
                // Command is telling us to aim
                // Arm should be lowered and latched
                // Unwind string to set arm angle and tension spring to set power
                // Check to make sure release pin latched
                if (isReleasePinReleased()) {
                    _state_prior = CatapultState.AIMING;
                    _state_error = "Pin released";
                    _state = CatapultState.ERROR;
                    return;
                }
                // TODO: Adjust arm and spring to aim
                return;
            case READY:
                // We have been lowered, locked, loaded, and aimed.
                // We're ready to shoot.
                // If we are not lowered, locked and loaded, we just shot a ball.
                if (!isArmLowered() && isReleasePinReleased() && !isBallDetected()) {
                    // Lower catapult arm to get ready again
                    _state = CatapultState.LOWERING_ARM;
                    return;
                }
                // If we are lowered and locked but not loaded, we need to load again
                if (isArmLowered() && isReleasePinLocked() && !isBallDetected()) {
                    _state = CatapultState.LOADING;
                    return;
                }
                // If we're detecting a ball when the arm is not lowered, we have
                // a problem -- a ball may have sneaked in under the catapult arm
                if (!isArmLowered() && isBallDetected()) {
                    _state_prior = CatapultState.AIMING;
                    _state_error = "Pin released";
                    _state = CatapultState.ERROR;
                    return;
                }
                // Wait for something to happen
                return;
            case WAITING:
                // Use this state to do nothing but wait
                return;
            case ERROR:
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);
                // TODO: Figure out what's wrong
                // TODO: Initiate automatic recovery procedure
                // TODO: Wait for commands to help us get out of our mess
                return;          
            default:
                // Neither catapult motor should be moving
                setArmMotorSpeed(0);
                setSpringMotorSpeed(0);            
                return;
        }

    }

    public void zeroSpringEncoder() {
        if (!_springEncoderZeroed) {
            _springMotor.setSelectedSensorPosition(SPRING_BOTTOM_LIMIT);
            _springEncoderZeroed = true;
        }
    }

    public void setSpringMotorSpeed(double speed) {
        _springMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setSpringDistance(double goalMeters){
        _springGoal = goalMeters;
        _springMotor.set(ControlMode.MotionMagic, (_springGoal / TICKS_TO_METERS));
    }

    public double getSpringEncoderTicks() {
        return _springMotor.getSelectedSensorPosition();
    }

    public double getSpringPosition() {
        return getSpringEncoderTicks() * TICKS_TO_METERS;
    }

    public boolean isSpringZeroed() {
        return _springEncoderZeroed;
    }

    public boolean isSpringAtPosition() {
        return Math.abs(getSpringPosition() - _springGoal) < SPRING_TOLERANCE;
    }

    public boolean isSpringHallTriggered() {
        return _springHall.get();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }

    public double getArmRotation() {
        return _armEncoder.getPosition();
    }

    // meters
    public double getArmStringLength() {
        return getArmRotation() * ARM_WINCH_DRUM_CIRCUMFERENCE;
    }

    public double getArmReleaseAngle() {
        return STOWED_ANGLE - stringLengthToAngle(getArmRotation() * ARM_WINCH_DRUM_CIRCUMFERENCE);
    }
    
    // meters and radians.
    protected double stringLengthToAngle(double stringLength) {
        return LINEAR_REGRESSION_SLOPE * stringLength + LINEAR_REGRESSION_OFFSET;
    }

    // radians
    protected double angleToStringLength(double angle) {
        return (angle - STOWED_ANGLE + LINEAR_REGRESSION_OFFSET) / LINEAR_REGRESSION_SLOPE;
    }

    public void zeroArmEncoder() {
        if (!_armEncoderZeroed) {
            _armEncoder.setPosition(WINCH_BOTTOM_LIMIT);
            _armEncoderZeroed = true;
        }
    }

    public boolean isArmZeroed() {
        return _armEncoderZeroed;
    }

    public double getArmControllerOutput() {
        return _armController.calculate(getArmStringLength());
    }

    public void setArmGoal(double stringLength) {
        _armController.setGoal(-stringLength);
    }

    public boolean isArmAtGoal() {
        return _armController.atGoal();
    }

    public boolean isArmLowered() {
        return _armHall.get();
    }

    public void lockArm() {
        _releasePin.set(PinPosition.LOCKED.getSolenoidValue());
    }

    public void releaseArm() {
        _releasePin.set(PinPosition.RELEASED.getSolenoidValue());
    }

    public boolean isReleasePinLocked() {
        return _releasePin.get() == PinPosition.LOCKED.getSolenoidValue();
    }

    public boolean isReleasePinReleased() {
        return _releasePin.get() == PinPosition.RELEASED.getSolenoidValue();
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

    // calculate linear regression.
    public double calculateIdealString(double dist) {
        return (-0.0081481481 * (dist * dist * dist)) +
                (0.1132539683 * (dist * dist))
                - (0.4818121693 * dist) + 0.9078174603;
    }

    // calculated from linear regression
    public double calculateIdealSpring(double dist) {
        return (0.0008888889 * (dist * dist * dist)) -
                (0.0143095238 * (dist * dist )) +
                (0.0820515873 * dist) - 0.0838690476;
    }

    public boolean isBallDetected() {
        return _proximitySensor.get();
    }

    public boolean isBlueBallDetected() {
        return _colorSensor.isBlue() && isBallDetected();
    }

    public boolean isRedBallDetected() {
        return _colorSensor.isRed() && isBallDetected();
    }

    public boolean isRedAlliance() {
        return _alliance == DriverStation.Alliance.Red;
    }
    public void raiseGate() {
        _gate.raise();
    }

    public void lowerGate() {
        _gate.lower();
    }

    public CatapultState getState() {
        return _state;
    }

    public void setState(CatapultState state) {
        _state = state;
    }

    public void setInitialized(boolean initialized) {
        _initialized = initialized;
    }

    public boolean isInitialized() {
        return _initialized;
    }
    
    @Override
    public void updateDashboard() {
        // Spring values
//        metric("Spring encoder rotations", getSpringEncoderRotation());
        metric("Spring position", getSpringPosition());
        metric("Spring goal ticks", _springGoal / TICKS_TO_METERS);
        metric("spring zeroed", _springEncoderZeroed);
        metric("Spring encoder ticks", getSpringEncoderTicks());
        metric("Spring motor output", _springMotor.getMotorOutputPercent());
        metric("Spring goal", _springGoal);
        metric("Spring Hall Effect", isSpringHallTriggered());

        // Winch values
        metric("Winch rotation", getArmRotation());
        metric("Winch controller output", _armMotor.getAppliedOutput());
        metric("winch goal", _armController.getGoal().position);
        metric("Winch string length", getArmStringLength());
//        metric("String length", stringLengthToAngle(getArmReleaseAngle()));
//        metric("Winch goal", Units.radiansToDegrees(stringLengthToAngle(_armController.getGoal().position)));
        metric("Blue", isBlueBallDetected());
        metric("Red", isRedBallDetected());
        // Catapult arm values
        metric("Arm state", _state.name());
        metric("Arm release angle", getArmReleaseAngle());
        metric("Arm Hall Effect", isArmLowered());
        metric("Ball detected", isBallDetected());
    }

    /**
     * Pass true here to trigger a shot from autonomous.
     * @param value
     */
    public void setAutoshoot(boolean value) {
        _autoShoot = value;
    }

    public boolean isAutoShoot() {
        return _autoShoot;
    }

    public void setSetpoint(CatapultSetpoint setpoint) {
        _setpoint = setpoint;
        info("Setting setpoint to " + setpoint.toString());
    }

    public CatapultSetpoint getSetpoint() {
        return _setpoint;
    }

    public enum CatapultSetpoint {
        NONE(0),
        NEAR(1),
        MID(2),
        FAR(3);

        private final int _value;
        CatapultSetpoint(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

    public void setStaticGoals() {
        switch(_setpoint) {
            case NEAR:
                setArmGoal(Auto.StaticShots.NEAR_WINCH);
                setSpringDistance(Auto.StaticShots.NEAR_SPRING);
                break;
            case MID:
                setArmGoal(Auto.StaticShots.MID_WINCH);
                setSpringDistance(Auto.StaticShots.MID_SPRING);
                break;
            case FAR:
                setArmGoal(Auto.StaticShots.FAR_WINCH);
                setSpringDistance(Auto.StaticShots.FAR_SPRING);
                break;
            default:
                setArmGoal(Auto.StaticShots.DEFAULT_WINCH);
                setSpringDistance(Auto.StaticShots.DEFAULT_SPRING);
                break;
        }
        
    }
   
    public String getError() {
        return _state_error + " during " + _state_prior.name();
    }
}