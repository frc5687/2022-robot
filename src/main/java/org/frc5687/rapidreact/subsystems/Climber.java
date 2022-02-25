/* Team 5687 (C)5687-2022 */
package org.frc5687.rapidreact.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAlternateEncoder;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.HallEffect;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Climber extends OutliersSubsystem{
    
    private CANSparkMax _stationaryArmWinch;
    private CANSparkMax _rockerArmWinch;
    private DoubleSolenoid _rocker;
    private ProfiledPIDController _staController;
    private ProfiledPIDController _rockController;
    private RelativeEncoder _stationaryArmWinchEncoder;
    private RelativeEncoder _rockerArmWinchEncoder;
    private HallEffect _staArmUp;
    private HallEffect _staArmDown;
    private HallEffect _rockArmUp;
    private HallEffect _rockArmDown;
    private ClimberStep _step = ClimberStep.UNKNOWN;

    private boolean _staControllerEnabled = false;
    private boolean _rockControllerEnabled = false;

    private double _staSpeed = 0.0;
    private double _rockSpeed = 0.0;

    public void setStep(ClimberStep step){
        //Sets the current step of the climbing process
        _step = step;
    }

    public Climber(OutliersContainer container) {
        super(container);

        logMetrics("Stationary/Position", "Stationary/Goal", "Stationary/Enabled", "Stationary/Speed", "Stationary/Up", "Stationary/Up", "Rocker/Position", "Rocker/Goal", "Rocker/Enabled", "Rocker/Speed", "Rocker/Up", "Rocker/Down", "Rocker Cylinder");

        _stationaryArmWinch = new CANSparkMax(RobotMap.CAN.SPARKMAX.STATIONARY_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rockerArmWinch = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROCKER_CLIMBER, CANSparkMax.MotorType.kBrushless);
        _rocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PCH.CLIMBER_IN, RobotMap.PCH.CLIMBER_OUT);

        _stationaryArmWinch.setIdleMode(IdleMode.kCoast);
        _stationaryArmWinch.setInverted(Constants.Climber.STATIONARY_ARM_REVERSED);
        _stationaryArmWinch.setSecondaryCurrentLimit(Constants.Climber.STATIONARY_ARM_CURRENT_LIMIT);
        _stationaryArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _stationaryArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _stationaryArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        _rockerArmWinch.setIdleMode(IdleMode.kCoast);
        _rockerArmWinch.setInverted(Constants.Climber.ROCKER_ARM_REVERSED);
        _rockerArmWinch.setSecondaryCurrentLimit(Constants.Climber.ROCKER_ARM_CURRENT_LIMIT);
        _rockerArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        _rockerArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        _rockerArmWinch.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        _stationaryArmWinch.burnFlash();
        _rockerArmWinch.burnFlash();

        _staArmUp = new HallEffect(RobotMap.DIO.STATIONARY_ARM_TOP_HALL);
        _staArmDown = new HallEffect(RobotMap.DIO.STATIONARY_ARM_BOTTOM_HALL);

        _rockArmUp = new HallEffect(RobotMap.DIO.ROCKER_ARM_TOP_HALL);
        _rockArmDown = new HallEffect(RobotMap.DIO.ROCKER_ARM_BOTTOM_HALL);

        _staController = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD,  new TrapezoidProfile.Constraints(1.5, 2.0));
        _rockController = new ProfiledPIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD, new TrapezoidProfile.Constraints(1.5, 2.0));

        _stationaryArmWinchEncoder = _stationaryArmWinch.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, Constants.Climber.COUNTS_PER_REVOLUTION);
        _stationaryArmWinchEncoder.setPositionConversionFactor(Constants.Climber.STATIONARY_ENCODER_CONVERSION_FACTOR);

        _rockerArmWinchEncoder = _rockerArmWinch.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, Constants.Climber.COUNTS_PER_REVOLUTION);
        _rockerArmWinchEncoder.setPositionConversionFactor(Constants.Climber.ROCKER_ENCODER_CONVERSION_FACTOR);

        _stationaryArmWinchEncoder.setPosition(0);
    }

    /**
     * Directly sets the speed of the stationary arm winch.
     * Note that this is private because it is intended to be called only by the climber's controllers. 
     * @param speed
     */
    private void setStaSpeed(double speed) {
        _staSpeed = speed;
        _stationaryArmWinch.set(speed);
    }

    /**
     * Directly sets the speed of the rocker arm winch.
     * Note that this is private because it is intended to be called only by the climber's controllers. 
     * @param speed
     */
    private void setRockSpeed(double speed) {
        _rockSpeed = speed;
        _rockerArmWinch.set(speed);
    }

    public void stop(){
        stopRockerArm();
        stopStationaryArm();
    }

    /**
     * Sets the goal of the PID controller
     * @param goal
     */
    public void setStaGoal(double goal){
        _staController.setGoal(goal);
        _staControllerEnabled = true;
    }


    public void setRockGoal(double goal){
        //Sets the goal of the PID controller
        _rockController.setGoal(goal);
    }

    public void rockerIn(){
        //Rock arm in
        _rocker.set(Value.kForward);
    }

    public void rockerOut(){
        //Rock arm out
        _rocker.set(Value.kReverse);
    }

    public void dropDriveSpeed(){
        //Drop driveTrain speed 
        // Constants.DriveTrain.MAX_MPS = Constants.Climber.CLIMB_DRIVE_SPEED;
    }

    public boolean isStaArmUp(){
        //Is the stationary arm up
        return _staArmUp.get();
    }

    public boolean isStaArmDown(){
        //Is the stationary arm down
        return _staArmDown.get();
    }

    public boolean isRockArmUp(){
        return _rockArmUp.get();
    }

    public boolean isRockArmDown(){
        return _rockArmDown.get();
    }


    public void zeroEncoder(){
        _stationaryArmWinchEncoder.setPosition(0);
    }

    public void invert(boolean inverted){
        _stationaryArmWinch.setInverted(inverted);
    }

    @Override
    public void updateDashboard() {
        metric("Stationary/Position", getStaPosition());
        metric("Stationary/Goal", _staController.getGoal().position);
        metric("Stationary/Enabled", _staControllerEnabled);
        metric("Stationary/Speed", _staSpeed); 
        metric("Stationary/Up", _staArmUp.get());
        metric("Stationary/Down", _staArmDown.get());

        
        metric("Rocker/Position", getRockPosition());
        metric("Rocker/Goal", _rockController.getGoal().position);
        metric("Rocker/Enabled", _rockControllerEnabled);
        metric("Rocker/Speed", _rockSpeed);
        metric("Rocker/Up", isRockArmUp()); 
        metric("Rocker/Down", isStaArmDown());
        metric("Rocker Cylinder", getRockerLabel());

        metric("Step", _step.name());

    }

    /**
     * Run the winch PID controllers.  Called by the various Climber Command classes.
     */
    public void runControllers() {
        if (_staControllerEnabled) {
            setStaSpeed(_staController.calculate(_stationaryArmWinchEncoder.getPosition()));
        }
        if (_rockControllerEnabled) {
            setRockSpeed(_rockController.calculate(_rockerArmWinchEncoder.getPosition()));
        }
    }

    public boolean getStaAtGoal() {
        return _staController.atGoal();
    }

    public boolean getRockAtGoal() {
        return _rockController.atGoal();
    }

    public void stopStationaryArm() {
        _staControllerEnabled = false;
        setStaSpeed(0.0);
        _stationaryArmWinch.setIdleMode(IdleMode.kBrake);
    }

    public void stopRockerArm() {
        _rockControllerEnabled = false;
        setRockSpeed(0.0);
        _rockerArmWinch.setIdleMode(IdleMode.kBrake);
    }

    public double getStaSpeed() {
        return _staSpeed;
    }

    public double getStaPosition() {
        return _stationaryArmWinchEncoder.getPosition();
    }

    public double getRockSpeed() {
        return _rockSpeed;
    }

    public double getRockPosition() {
        return _rockerArmWinchEncoder.getPosition();
    }

    public String getRockerLabel() {
        return(_rocker.get() == Value.kForward) ? "In" : "Out";
    }

    public enum ClimberStep {
        UNKNOWN(0),
        STOW(1),
        PREP_TO_CLIMB(2),
        ATTACH_MID(2),
        ATTACH_HIGH(3),
        ATTACH_TRAVERSAL(4);

        private final int _value;
        ClimberStep(int value) { 
            _value = value; 
        }

        public int getValue() { 
            return _value; 
        }
    }

}