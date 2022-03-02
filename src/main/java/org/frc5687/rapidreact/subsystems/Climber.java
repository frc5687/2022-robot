/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * The Climber subsytem consists of two hooked arms on spring-loaded cylinders, restrained by winches.
 * Paying the winch out allows the hook to extend, retracting the winch pulls the hook down.
 * One arm is stationary, the other rocks foreward and back driven by a pneumatic piston.
 * 
 * The winches are driven by baby neos on sparkmax controllers (with integral encoders) and the each arm
 * has a pair of hall-effect sensors to indicate when they are UP or DOWN.
 * 
 * The winches are controlled by ProfiledPIDControllers, with goals set using setStaGoal and setRockGoal.
 * The controllers are enabled/disabled using the enable
 */
public class Climber extends OutliersSubsystem{
    
    private TalonFX _stationaryArmWinch;
    private TalonFX _rockerArmWinch;
    private DoubleSolenoid _rocker;
    private ProfiledPIDController _staController;
    private ProfiledPIDController _rockController;
    private CANCoder _stationaryArmWinchEncoder;
    private CANCoder _rockerArmWinchEncoder;
    private HallEffect _staArmUp;
    private HallEffect _staArmDown;
    private HallEffect _rockArmUp;
    private HallEffect _rockArmDown;
    private ClimberStep _step = ClimberStep.UNKNOWN;
    private boolean _climberStopped = true;

    private boolean _staControllerEnabled = false;
    private boolean _rockControllerEnabled = false;

    private double _staSpeed = 0.0;
    private double _rockSpeed = 0.0;

    public void setStep(ClimberStep step){
        //Sets the current step of the climbing process
        _step = step;
    }
 
    public ClimberStep getStep() {
        return _step;
    }
    
    public Climber(OutliersContainer container) {
        super(container);

        logMetrics("Stationary/Position", "Stationary/Goal", "Stationary/Enabled", "Stationary/Speed", "Stationary/Up", "Stationary/Up", "Rocker/Position", "Rocker/Goal", "Rocker/Enabled", "Rocker/Speed", "Rocker/Up", "Rocker/Down", "Rocker Cylinder");

        _stationaryArmWinch = new TalonFX(RobotMap.CAN.TALONFX.STATIONARY_CLIMBER);
        _rockerArmWinch = new TalonFX(RobotMap.CAN.TALONFX.ROCKER_CLIMBER);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.CLIMBER_IN, RobotMap.PCH.CLIMBER_OUT);

        _stationaryArmWinch.setInverted(Constants.Climber.STATIONARY_ARM_REVERSED);
        _stationaryArmWinch.setNeutralMode(NeutralMode.Coast);
        _stationaryArmWinch.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));


        _rockerArmWinch.setInverted(Constants.Climber.ROCKER_ENCODER_INVERTED);
        _rockerArmWinch.setNeutralMode(NeutralMode.Coast);
        _stationaryArmWinch.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
       


        _staArmUp = new HallEffect(RobotMap.DIO.STATIONARY_ARM_TOP_HALL);
        _staArmDown = new HallEffect(RobotMap.DIO.STATIONARY_ARM_BOTTOM_HALL);

        _rockArmUp = new HallEffect(RobotMap.DIO.ROCKER_ARM_TOP_HALL);
        _rockArmDown = new HallEffect(RobotMap.DIO.ROCKER_ARM_BOTTOM_HALL);

        _staController = new ProfiledPIDController(
            Constants.Climber.kP, 
            Constants.Climber.kI,
            Constants.Climber.kD,  
            new TrapezoidProfile.Constraints(
                Constants.Climber.MAX_VELOCITY_MPS,
                Constants.Climber.MAX_ACCELERATION_MPSS
            )
        );
        _staController.setTolerance(Constants.Climber.ARM_TOLERANCE);
        _staController.setIntegratorRange(-Constants.Climber.ARM_IZONE, Constants.Climber.ARM_IZONE);

        _rockController = new ProfiledPIDController(
            Constants.Climber.kP,
            Constants.Climber.kI, 
            Constants.Climber.kD,
            new TrapezoidProfile.Constraints(
                    Constants.Climber.MAX_VELOCITY_MPS,
                    Constants.Climber.MAX_ACCELERATION_MPSS
            )
        );
        _rockController.setTolerance(Constants.Climber.ARM_TOLERANCE);
        _rockController.setIntegratorRange(-Constants.Climber.ARM_IZONE, Constants.Climber.ARM_IZONE);

        _stationaryArmWinchEncoder = new CANCoder(RobotMap.CAN.TALONFX.STATIONARY_CLIMBER);
        _rockerArmWinchEncoder = new CANCoder(RobotMap.CAN.TALONFX.ROCKER_CLIMBER);
    }

    /**
     * Directly sets the speed of the stationary arm winch.
     * Note that this is intended to be called only by the climber's controllers. 
     * @param speed
     */
    public void setStaSpeed(double speed) {
        _staSpeed = speed;
        _stationaryArmWinch.set(TalonFXControlMode.Position, speed);;
    }

    /**
     * Directly sets the speed of the rocker arm winch.
     * Note that this is intended to be called only by the climber's controllers. 
     * @param speed
     */
    public void setRockSpeed(double speed) {
        _rockSpeed = speed;
        _rockerArmWinch.set(TalonFXControlMode.Position, speed);
        info("Rocker arm speed set to " + speed);
    }


    /**
     * Disables both PID controllers and sets both winch motors to brake mode.
     */
    public void stop(){
        stopRockerArm();
        stopStationaryArm();
    }

    /**
     * Stops the stationary arm by disabling the PID controller, setting the winch speed to 0, and setting brake mode.
     */
    public void stopStationaryArm() {
        info("Stopping StationaryArm");
        _staControllerEnabled = false;
        setStaSpeed(0.0);
        _stationaryArmWinch.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Stops the rocker arm by disabling the PID controller, setting the winch speed to 0, and setting brake mode.
     */
    public void stopRockerArm() {
        info("Stopping RockerArm");
        _rockControllerEnabled = false;
        setRockSpeed(0.0);
        _rockerArmWinch.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the goal of the stationary arm PID controller
     * @param goalMeters in inches
     */
    public void setStaGoalMeters(double goalMeters){
        info("Setting StationaryArm goal to " + goalMeters);
        _staController.setGoal(-goalMeters);
        _staControllerEnabled = true;
    }


    /**
     * Sets the goal of the rocker arm PID controller
     * @param goalMeters in inches
     */
    public void setRockGoalMeters(double goalMeters){
        info("Setting RockerArm goal to " + goalMeters);
        //Sets the goal of the PID controller
        _rockController.setGoal(-goalMeters);
        _rockControllerEnabled = true;
    }


    /**
     * Gets the position of the stationary arm in meters
     * @return
     */
    public double getStaPositionMeters(){
        return _stationaryArmWinch.getSelectedSensorPosition() * Constants.Climber.ENCODER_RATIO;
    }

    /**
     * Get the rocker arm position in meters
     * @return
     */
    public double getRockPositionMeters(){
        return _rockerArmWinch.getSelectedSensorPosition() * Constants.Climber.ENCODER_RATIO;
    }

    /**
     * Run the winch PID controllers.  Called once per cycle by the various Climber Command classes.
     * Using the rocker arm and the stationary arms encoders with the string to get the distance
     * the arm is extened, that is used as the input to the PID controllers
     */
    public void runControllers() {
        if (_staControllerEnabled) {
            metric("Running stationary controller", true);
            setStaSpeed(_staController.calculate(getStaPositionMeters()));
        }
        if (_rockControllerEnabled) {
            metric("Running rocker controller", true);
            setRockSpeed(_rockController.calculate(getRockPositionMeters()));
        }
    }

    /**
     * Pushes the rocker-arm piston in.
     */
    public void rockerIn(){
        //Rock arm in
        info("Setting Rocker in");

        _rocker.set(Value.kForward);
    }

    /**
     * Pushes the rocker-arm piston out.
     */
    public void rockerOut(){
        info("Setting Rocker out");
        //Rock arm out
        _rocker.set(Value.kReverse);
    }

    public void dropDriveSpeed(){
        //Drop driveTrain speed 
        // Constants.DriveTrain.MAX_MPS = Constants.Climber.CLIMB_DRIVE_SPEED;
    }

    /**
     * Checks the stationary arm "up" hall-effect sensor.
     * @return true if the sensor is triggered
     */
    public boolean isStaArmUp(){
        return _staArmUp.get();
    }

    /**
     * Checks the stationary arm "down" hall-effect sensor.
     * @return true if the sensor is triggered or if the arm is stalled
     */
    public boolean isStaArmDown(){
        return _staArmDown.get() || isStaArmStalled();
    }

    /**
     * Checks the rocker arm "up" hall-effect sensor.
     * @return true if the sensor is triggered
     */
    public boolean isRockArmUp(){
        return _rockArmUp.get()
        || isRockArmStalled();
    }


    private int _staStallCycles = 0;
    private int _rockStallCycles = 0;

    /**
     * Checks to see if the stationary arm motor is stalled.
     * It is stalled if the amperage draw is > Constant and it has not moved in more than Constant cycles
     */
    public boolean isStaArmStalled() {
        if (isArmStalled(_stationaryArmWinch, _stationaryArmWinchEncoder)) {
            _staStallCycles++;
        } else {
            _staStallCycles=0;
        }
        return _staStallCycles > Constants.Climber.MAX_STALL_CYCLES;
    }

    /**
     * Checks to see if the rocker arm motor is stalled.
     * It is stalled if the amperage draw is > STALL_CURRENT and it has not moved in more than MAX_STALL_CYCLES cycles
     */
    public boolean isRockArmStalled() {
        if (isArmStalled(_rockerArmWinch, _rockerArmWinchEncoder)) {
            _rockStallCycles++;
        } else {
            _rockStallCycles=0;
        }
        return _rockStallCycles > Constants.Climber.MAX_STALL_CYCLES;
    }

    /** 
     * Checks to see if an arm is stalling.  An arm is stalling if the controller is sending more than STALL_CURRENT current
     * but the encoder is reading less that STALL_MIN_RPM rotations.
     */
    private boolean isArmStalled(TalonFX controller, CANCoder encoder) {
        return controller.getSupplyCurrent() > Constants.Climber.STALL_CURRENT
            && Math.abs(encoder.getVelocity()) < Constants.Climber.STALL_MIN_RPM;  
    }

    /**
     * Checks the rocker arm "down" hall-effect sensor.
     * @return true if the sensor is triggered or the arm motor stalled
     */
    public boolean isRockArmDown(){
        return _rockArmDown.get() || isRockArmStalled();
    }

    /**
     * Sets the stationary arm encoder to 0.  This should be called when the "down" HE is triggered.
     */
    public void zeroStationaryArmEncoder(){
        _stationaryArmWinchEncoder.setPosition(0);
    }

    /**
     * Sets the rocker arm encoder to 0.  This should be called when the "down" HE is triggered.
     */
    public void zeroRockerArmEncoder() {
        _rockerArmWinchEncoder.setPosition(0);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary Arm temp", _stationaryArmWinch.getTemperature());
        metric("Rocker Arm temp", _rockerArmWinch.getTemperature());
        metric("Stationary/Position", getStaPositionMeters());
        metric("Stationary/Goal", _staController.getGoal().position);
        metric("Stationary/Enabled", _staControllerEnabled);
        metric("Stationary/Encoder", _stationaryArmWinchEncoder.getPosition());
        metric("Stationary/Speed", _staSpeed); 
        metric("Stationary/Up", _staArmUp.get());
        metric("Stationary/Down", _staArmDown.get());
        metric("Rocker/Position", getRockPositionMeters());
        metric("Rocker/Goal", _rockController.getGoal().position);
        metric("Rocker/Enabled", _rockControllerEnabled);
        metric("Rocker/Speed", _rockSpeed);
        metric("Rocker/Up", isRockArmUp()); 
        metric("Rocker/Encoder", _stationaryArmWinchEncoder.getPosition());
        metric("Rocker/Down", isRockArmDown());
        metric("Rocker Cylinder", getRockerLabel());

        metric("Step", _step.name());
        metric("Climber stopped", _climberStopped);
    }


    /**
     * Checks whether the stationary arm PID controlled is at its goal.
     * @return true if within tolerance
     */
    public boolean isStaAtGoal() {
        return _staController.atGoal();
    }

    /**
     * Checks whether the rocker arm PID controlled is at its goal.
     * @return true if within tolerance
     */
    public boolean isRockAtGoal() {
        return _rockController.atGoal();
    }


    /**
     * @return the current stationary arm winch speed.
     */
    public double getStaSpeed() {
        return _staSpeed;
    }


    /**
     * @return the current rocker arm winch speed.
     */
    public double getRockSpeed() {
        return _rockSpeed;
    }

    /**
     * @return the position (In or Out) of the rocker piston
     */
    public String getRockerLabel() {
        return(_rocker.get() == Value.kForward) ? "In" : "Out";
    }

    public enum ClimberStep {
        UNKNOWN(0),
        STOW(1),
        PREP_TO_CLIMB(2),
        ATTACH_MID(2),
        ATTACH_HIGH(3),
        ATTACH_TRAVERSAL(4),
        DONE(5);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}