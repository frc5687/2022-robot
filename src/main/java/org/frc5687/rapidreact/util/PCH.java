package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PCH {

    private Compressor comp;  
    
    public PCH(){
        comp = new Compressor(14, PneumaticsModuleType.REVPH);
        comp.enableDigital();
    }

    public boolean isEnabled(){
        return comp.enabled();
    }
}
