package org.frc5687.rapidreact.util;

import com.revrobotics.ColorSensorV3;
import org.frc5687.rapidreact.Constants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor {
    
    private ColorSensorV3 _colorSensor;
    private I2C.Port _port;
    private final ColorMatch _colorMatcher;
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSensor(I2C.Port port){
        _port = port;
        _colorSensor = new ColorSensorV3(_port);
        _colorMatcher = new ColorMatch();
        _colorMatcher.addColorMatch(kBlueTarget);
        _colorMatcher.addColorMatch(kRedTarget);
    }

    public double getIR(){
        //Returns to raw amount of IR light detected
        return _colorSensor.getIR();
    }

    //Get the raw amounts of Red, Green, Blue detected
    public double getRed(){
        return _colorSensor.getRed();
    }

    public String getColor(){
        Color detectedColor = _colorSensor.getColor();
        String colorString;
        ColorMatchResult match = _colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kBlueTarget) {
            colorString = "Blue";
            SmartDashboard.putString("Color String", colorString);
            return colorString;
          } else if (match.color == kRedTarget) {
            colorString = "Red";
            SmartDashboard.putString("Color String", colorString);
            return colorString;
          } else {
            colorString = "Unknown";
            SmartDashboard.putString("Color String", colorString);
            return colorString;
        }
    }

    public boolean isBlue(){
        return getColor().equals("Blue");
    }

    public boolean isRed(){
        return getColor().equals("Red");
    }

    public boolean isGreen(){
        return getColor().equals("Green");
    }

    public double getGreen(){
        return _colorSensor.getGreen();
    }

    public double getBlue(){
        return _colorSensor.getBlue();
    }

    public double getProximity(){
        //Get the robots proximity
        return _colorSensor.getProximity();
    }

    public boolean hasBall(){
        return !(getProximity() < Constants.ColorSensor.COLOR_PROXIMITY_BUFFER);
    }

    public void updateDashboard(){
        SmartDashboard.putBoolean("Red Ball", isRed());
        SmartDashboard.putBoolean("Blue Ball", isBlue());
        SmartDashboard.putNumber("IR", getIR());
        SmartDashboard.putNumber("Proximity", getProximity());
    }
}
