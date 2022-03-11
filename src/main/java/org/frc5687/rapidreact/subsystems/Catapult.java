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
import org.frc5687.rapidreact.util.ColorSensor;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.util.ServoStop;

import static org.frc5687.rapidreact.Constants.Catapult.*;

public class Catapult extends OutliersSubsystem {

    private final TalonFX _springMotor;
    private final CANSparkMax _winchMotor;

    private final RelativeEncoder _winchEncoder;

    private final ProfiledPIDController _winchController;

    private final DoubleSolenoid _releasePin;

    private final HallEffect _springHall;
    private final HallEffect _armHall;

    private boolean _springEncoderZeroed = false;
    private boolean _winchEncoderZeroed = false;

    private CatapultState _state;

    private ColorSensor _colorSensor;
    private ServoStop _gate;

    private DriverStation.Alliance _alliance;

    private double _springGoal;

    private boolean _autoShoot;

    private boolean _initialized = false;

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
        DEBUG(8),
        // a button will stop all catapult movement, this is for the case if
        // a ball gets under the catapult.
        KILL(9),

        AUTO(10);


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

        // Winch motor
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor.restoreFactoryDefaults();
        _winchMotor.setInverted(WINCH_MOTOR_INVERTED);
        _winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        _winchMotor.setSmartCurrentLimit(WINCH_CURRENT_LIMIT);


        // Pneumatics (catapult locking pin)
        _releasePin = new DoubleSolenoid(PneumaticsModuleType.REVPH,
                RobotMap.PCH.RELEASE_PIN_HIGH,
                RobotMap.PCH.RELEASE_PIN_LOW);

        // Hall effects (sense position of springs and catapult arm)
        _springHall = new HallEffect(RobotMap.DIO.SPRING_HALL_EFFECT);
        _armHall = new HallEffect(RobotMap.DIO.ARM_HALL_EFFECT);

        // Encoders
        _winchEncoder = _winchMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, COUNTS_PER_REVOLUTION);


        // PID controllers
        _springMotor.config_kP(MOTION_MAGIC_SLOT, SPRING_kP);
        _springMotor.config_kI(MOTION_MAGIC_SLOT, SPRING_kI);
        _springMotor.config_kD(MOTION_MAGIC_SLOT, SPRING_kD);
        _springMotor.config_kF(MOTION_MAGIC_SLOT, SPRING_kF);

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

        //Save changes into flash memory of spark maxes
        _winchMotor.burnFlash();
        // set state
        _springEncoderZeroed = false;
        _winchEncoderZeroed = false;
        _state = CatapultState.DEBUG;
        _gate = new ServoStop(RobotMap.PWM.INTAKE_STOPPER);
        _colorSensor = new ColorSensor(I2C.Port.kMXP);
        _alliance = DriverStation.getAlliance();
        _springGoal = 0;
    }


    @Override
    public void periodic() {
        super.periodic();

        if (isArmLowered() && (_winchMotor.getAppliedOutput() > 0)) {
            setWinchMotorSpeed(0);
//            setWinchGoal(0);
        }
        if (_winchMotor.getOutputCurrent() > WINCH_CURRENT_LIMIT) {
            setWinchMotorSpeed(0);
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

    public boolean isSpringHallTriggered() { return _springHall.get(); }


    public void setWinchMotorSpeed(double speed) {
        _winchMotor.set(speed);
    }

    public double getWinchRotation() {
        return _winchEncoder.getPosition();
    }

    // meters
    public double getWinchStringLength() {
        return getWinchRotation() * ARM_WINCH_DRUM_CIRCUMFERENCE;
    }

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

    public void zeroWinchEncoder() {
        if (!_winchEncoderZeroed) {
            _winchEncoder.setPosition(WINCH_BOTTOM_LIMIT);
            _winchEncoderZeroed = true;
        }
    }
    public boolean isWinchZeroed() {
        return _winchEncoderZeroed;
    }

    public double getWinchControllerOutput() {
        return _winchController.calculate(getWinchStringLength());
    }

    public void setWinchGoal(double stringLength) {
        _winchController.setGoal(-stringLength);
    }

    public boolean isWinchAtGoal() {
        return _winchController.atGoal();
    }

    public boolean isArmLowered() { return _armHall.get(); }


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

    public boolean isReleasePinLocked() {
        return _releasePin.get() == PinPosition.LOCKED.getSolenoidValue();
    }
    public boolean isReleasePinReleased() {
        return _releasePin.get() == PinPosition.RELEASED.getSolenoidValue();
    }
    public boolean isBlueBallDetected() {
        return _colorSensor.isBlue() && _colorSensor.hasBall();
    }

    public boolean isRedBallDetected() {
        return _colorSensor.isRed() && _colorSensor.hasBall();
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
        metric("Winch rotation", getWinchRotation());
        metric("Winch controller output", _winchMotor.getAppliedOutput());
        metric("winch goal", _winchController.getGoal().position);
        metric("Winch string length", getWinchStringLength());
//        metric("String length", stringLengthToAngle(getArmReleaseAngle()));
//        metric("Winch goal", Units.radiansToDegrees(stringLengthToAngle(_winchController.getGoal().position)));
        metric("Blue", isBlueBallDetected());
        metric("Red", isRedBallDetected());
        // Catapult arm values
        metric("Arm state", _state.name());
        metric("Arm release angle", getArmReleaseAngle());
        metric("Arm Hall Effect", isArmLowered());
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
}