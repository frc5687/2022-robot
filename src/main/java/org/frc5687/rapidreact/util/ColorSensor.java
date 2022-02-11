package org.frc5687.rapidreact.util;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSensor {
    
    private ColorSensorV3 _color;
    private I2C.Port _port;

    public ColorSensor(){
        _port = I2C.Port.kMXP;
        _color = new ColorSensorV3(_port);
    }

    public double getIR(){
        //Returns to raw amount of IR light detected
        return _color.getIR();
    }

    //Get the raw amounts of Red, Green, Blue detected
    public double getRed(){
        return _color.getRed();
    }

    public double getGreen(){
        return _color.getGreen();
    }

    public double getBlue(){
        return _color.getBlue();
    }

    public double getPorximity(){
        //Get the robots proximity
        return _color.getProximity();
    }

    public void updateDashboard(){
        SmartDashboard.putNumber("Red", getRed());
        SmartDashboard.putNumber("Green", getRed());
        SmartDashboard.putNumber("Blue", getBlue());
        SmartDashboard.putNumber("IR", getIR());
        SmartDashboard.putNumber("Proximity", getPorximity());
        SmartDashboard.putString("Color", _color.getColor().toString());
    }
}
