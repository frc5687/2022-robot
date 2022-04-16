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
import org.frc5687.rapidreact.util.*;
import org.frc5687.rapidreact.subsystems.Indexer;
import static org.frc5687.rapidreact.Constants.Catapult.*;

public class Catapult extends OutliersSubsystem {

    private final TalonFX _springMotor;
    private final CANSparkMax _winchMotor;
    private final DoubleSolenoid _releasePin;

    private final RelativeEncoder _winchEncoder;
    private final HallEffect _springHall;
    private final HallEffect _armHall;

    private Indexer _indexer;

    private final ProfiledPIDController _winchController;

    private boolean _springEncoderZeroed = false;
    private boolean _winchEncoderZeroed = false;
    private CatapultState _state;
    private double _springGoal;
    private double _winchGoal;
    private boolean _autonomShoot;
    private boolean _automatShoot;
    private boolean _initialized = false;
    private CatapultSetpoint _setpoint = CatapultSetpoint.NONE;

    public void setState(CatapultState state) {
        _state = state;
    }

    /** Catapult constructor */
    public Catapult(OutliersContainer container) {
        super(container);
        // Motor controllers (Spark Maxes)
        // Spring motor
        _springMotor = new TalonFX(RobotMap.CAN.TALONFX.CATAPULT_SPRING);
        _springMotor .setNeutralMode(NeutralMode.Brake);
        _springMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _springMotor.setInverted(SPRING_MOTOR_INVERTED);
        _springMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        _springMotor.configMotionAcceleration(ACCELERATION);
        _springMotor.configVoltageMeasurementFilter(8);
        _springMotor.enableVoltageCompensation(true);
        _springMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, CTRE_FRAME_PERIOD, CTRE_TIMEOUT);
        _springMotor.configClosedloopRamp(0, CTRE_TIMEOUT);

        // Winch motor
        _winchMotor = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_BABY_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchMotor.restoreFactoryDefaults();
        _winchMotor.setInverted(WINCH_MOTOR_INVERTED);
        _winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, CTRE_TIMEOUT);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, CTRE_TIMEOUT);
        _winchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, CTRE_TIMEOUT);
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

        // Proximity sensor (sense if ball on catapult arm)

        // PID controllers
        _springMotor.config_kP(MOTION_MAGIC_SLOT, SPRING_kP);
        _springMotor.config_kI(MOTION_MAGIC_SLOT, SPRING_kI);
        _springMotor.config_kD(MOTION_MAGIC_SLOT, SPRING_kD);
        _springMotor.config_kF(MOTION_MAGIC_SLOT, SPRING_kF);
        _springMotor.config_IntegralZone(MOTION_MAGIC_SLOT, SPRING_IZONE);

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
        _springGoal = 0;
        _winchGoal = 0;
        _automatShoot = true;
        _autonomShoot = false;
    }


    @Override
    public void periodic() {
        super.periodic();
        if (isArmLowered() && (_winchMotor.getAppliedOutput() > 0)) {
            setWinchMotorSpeed(0);
        }
        if (_winchMotor.getOutputCurrent() > WINCH_CURRENT_LIMIT) {
            setWinchMotorSpeed(0);
        }
    }

    public void zeroSpringEncoder() {
        if (!_springEncoderZeroed) {
            // this is zero now not the real value. Don't want to mess with the regression.
            _springMotor.setSelectedSensorPosition(0);
            _springEncoderZeroed = true;
        }
    }

    public boolean isSpringZeroed() {
        return _springEncoderZeroed;
    }

    public double getSpringEncoderTicks() {
        return _springMotor.getSelectedSensorPosition();
    }

    public void setSpringMotorSpeed(double speed) {
        _springMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setSpringDistance(double goalMeters){
        _springGoal = goalMeters;
        _springMotor.set(ControlMode.MotionMagic, (_springGoal / TICKS_TO_METERS));
    }

    public double getSpringPosition() {
        return getSpringEncoderTicks() * TICKS_TO_METERS;
    }

    public boolean isSpringAtPosition() {
        return Math.abs(getSpringPosition() - _springGoal) < SPRING_TOLERANCE;
    }

    public boolean isSpringHallTriggered() { return _springHall.get(); }

    public double getSpringGoal() {
        return _springGoal;
    }

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
        return stringLengthToAngle(getWinchStringLength()) - STOWED_ANGLE;
    }

    public double calculateLeadExitVelocity(double currentExitVelocity, double[] targetVelocity) {
        if (targetVelocity[0] > 0) {
            // moving away from target
            return currentExitVelocity + Helpers.magnitude(targetVelocity);
        }
        return currentExitVelocity - Helpers.magnitude(targetVelocity);
    }

    public double calculateAngleToHitTarget(double exitVel, double xPosition, double zPosition) {
        double g = 9.81;
        double root = Math.pow(exitVel, 4) -
                g * (g * (xPosition * xPosition) +
                2 * zPosition * (exitVel * exitVel));
        if (root > 0) {
            double num = (exitVel * exitVel) + Math.sqrt(root);
            double num1 = (exitVel * exitVel) - Math.sqrt(root);
            double denom = g * xPosition;
            double angle1 = Math.atan(num / denom);
            double angle2 = Math.atan(num1 / denom);
            return Math.min(angle1, angle2);
        }
        return 0;
    }

    public double calculateLeadSpringDisplacement(double[] targetPosition, double[] targetVelocity) {
        double exitVel = calculateExitVelocity(getArmReleaseAngle(), _springGoal);
        double newExitVel = calculateLeadExitVelocity(exitVel, targetVelocity);
        double angleDisplacement = calculateAngleToHitTarget(newExitVel, targetPosition[0], targetPosition[2]) + STOWED_ANGLE;
        return exitVelocityToSpringDisplacement(newExitVel, angleDisplacement);
    }

    public double calculateLeadWinchString(double[] targetPosition, double[] targetVelocity) {
        double exitVel = calculateExitVelocity(getArmReleaseAngle(), _springGoal);
        double newExitVel = calculateLeadExitVelocity(exitVel, targetVelocity);
        double angle = calculateAngleToHitTarget(newExitVel, targetPosition[0], targetPosition[2]);
        return angleToStringLength(angle);
    }

    //meters and radians.
    protected double stringLengthToAngle(double stringLength) {
        return LINEAR_REGRESSION_SLOPE * stringLength + LINEAR_REGRESSION_OFFSET;
    }

    // radians
    protected double angleToStringLength(double angle) {
        return (angle + STOWED_ANGLE + LINEAR_REGRESSION_OFFSET) / LINEAR_REGRESSION_SLOPE;
    }

    protected double exitVelocityToSpringDisplacement(double exitVelocity, double angleDisplacement) {
        double angularAcceleration = (2.0 * (exitVelocity * exitVelocity)) /
                (angleDisplacement * (ARM_LENGTH * ARM_LENGTH));
        double torque = angularAcceleration * INERTIA_OF_ARM;
        double force = torque / LEVER_ARM_LENGTH;
        return force / SPRING_RATE;
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

    // calculated the ball exit velocity in meters per sec from the angle displacement and spring disp.
    public double calculateExitVelocity(double exitAngle, double spring) {
        double springDisplacement = Math.abs(spring - SPRING_BOTTOM_LIMIT);
        double angleDisplacement = Math.abs(exitAngle + STOWED_ANGLE);
        double force = springDisplacement * SPRING_RATE;
        double torque = LEVER_ARM_LENGTH * force;
        double angularAcceleration = torque / INERTIA_OF_ARM;
        double time = Math.sqrt((2.0 / angularAcceleration) * angleDisplacement);
        double angularVelocity = angleDisplacement / time;
        return angularVelocity * ARM_LENGTH;
    }

    // calculate linear regression.
    public double calculateIdealString(double dist) {
        // new curve
        return (SPRING_CUBIC_COEFF * (dist * dist * dist)) +
                (SPRING_SQUARE_COEFF * (dist * dist)) +
                (SPRING_LINEAR_COEFF * dist) + SPRING_OFFSET_COEFF;
        // old curve
//        return (0.005596480 * (dist * dist * dist)) -
//                (0.095972797 * (dist * dist)) +
//                (0.550428512 * dist) - 0.738967199;
    }
    // calculated from linear regression
    public double calculateIdealSpring(double dist) {
        // new curve
        return (WINCH_CUBIC_COEFF * (dist * dist * dist)) +
                (WINCH_SQUARE_COEFF * (dist * dist)) +
                (WINCH_LINEAR_COEFF * dist) + WINCH_OFFSET_COEFF;
        // old curve
//        return (0.000286128 * (dist * dist * dist)) -
//                (0.003328233 * (dist * dist)) +
//                (0.022190998 * dist) + 0.018160169;
    }

    public boolean isReleasePinLocked() {
        return _releasePin.get() == PinPosition.LOCKED.getSolenoidValue();
    }

    public boolean isReleasePinReleased() {
        return _releasePin.get() == PinPosition.RELEASED.getSolenoidValue();
    }

    public CatapultState getState() {
        return _state;
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
        metric("Spring position", getSpringPosition());
        metric("Spring goal ticks", _springGoal / TICKS_TO_METERS);
        metric("spring zeroed", _springEncoderZeroed);

//        metric("Spring encoder ticks", getSpringEncoderTicks());

        metric("Spring motor output", _springMotor.getMotorOutputPercent());
        metric("Spring goal", _springGoal);

//        metric("Spring Hall Effect", isSpringHallTriggered());

        // Winch values
//        metric("Winch rotation", getWinchRotation());
        metric("Winch controller output", _winchMotor.getAppliedOutput());
        metric("winch goal", _winchController.getGoal().position);
        metric("Winch string length", getWinchStringLength());
        // Catapult arm values
        metric("Arm state", _state.name());
//        metric("Arm release angle", getArmReleaseAngle());
//        metric("Arm Hall Effect", isArmLowered());
    }

    public boolean getAutomatShoot() {
        return _automatShoot;
    }

    public boolean getAutonomShoot() {
        return _autonomShoot;
    }

    public void setAutomatShoot(boolean value) {
        _automatShoot = value;
    }
    /**
     * Pass true here to trigger a shot from autonomous.
     * @param value
     */
    public void setAutonomShoot(boolean value) {
        _autonomShoot = value;
    }

    public void toggleAutomatShoot() {
        if (getAutomatShoot() == true) {
            setAutomatShoot(false);
        } else if (getAutomatShoot() == false) {
            setAutomatShoot(true);
        } else {
            setAutomatShoot(false);
        }
    }

    public void setSetpoint(CatapultSetpoint setpoint) {
        _setpoint = setpoint;
//        info("Setting setpoint to " + setpoint.toString());
    }

    public CatapultSetpoint getSetpoint() {
        return _setpoint;
    }

    public enum CatapultSetpoint {
        NONE(0),
        TARMAC(1),
        NEAR(2),
        MID(3),
        FAR(4);

        private final int _value;
        CatapultSetpoint(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

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

        WAIT_LOADING(2),
        // LOADING locks pin, check for which color ball we have.
        // Depending on the ball the robot enters the AIMING state or WRONG_BALL state.
        LOADING(3),

        // AIMING waited for the OI aim button to change states.
        // Will constantly be changing the spring and winch when moving in this state.
        // to automatically get the ball in no mater the distance.
        // Also have an override button if vision is not working.
        // Change state to SHOOTING.
        AIMING(4),


        // Set the winch goal and spring goal.
        // SHOOTING waits for shoot button to be pressed and the goals to be in tolerance.
        // If shoot button pressed, releases pin
        // then enters LOWERING_ARM
        SHOOTING(5),
        // wait for the shot, so we are sure arm is up and hall is not triggered
        // If hall doesn't work use time delay.
        WAIT_SHOT(6),

        // If intake is up lock the catapult;
        LOCK_OUT(7),

        // if in debug, set the winch and spring settings for initial position
        PRELOAD(8),
        // Until we have figured out catapult, start in DEBUG state
        // Check that everything looks good, then press
        // button to get into ZEROING state
        // Allow manual pin release and pin lock
        DEBUG(9),
        // a button will stop all catapult movement, this is for the case if
        // a ball gets under the catapult.
        KILL(10),

        AUTO(11);


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

    public void setStaticGoals() {
        switch(_setpoint) {
            case TARMAC:
                setWinchGoal(Auto.StaticShots.TARMAC_WINCH);
                setSpringDistance(Auto.StaticShots.TARMAC_SPRING);
                break;
            case NEAR:
                setWinchGoal(Auto.StaticShots.NEAR_WINCH);
                setSpringDistance(Auto.StaticShots.NEAR_SPRING);
                break;
            case MID:
                setWinchGoal(Auto.StaticShots.MID_WINCH);
                setSpringDistance(Auto.StaticShots.MID_SPRING);
                break;
            case FAR:
                setWinchGoal(Auto.StaticShots.FAR_WINCH);
                setSpringDistance(Auto.StaticShots.FAR_SPRING);
                break;
            default:
                setWinchGoal(Auto.StaticShots.DEFAULT_WINCH);
                setSpringDistance(Auto.StaticShots.DEFAULT_SPRING);
                break;
        }
        
    }

    
}