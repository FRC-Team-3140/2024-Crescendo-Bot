package frc.robot.libs;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AbsoluteEncoder extends AnalogEncoder {
    public AbsoluteEncoder(int analogID){
        super(analogID);
    }
    @Override
    public double getAbsolutePosition() {
        return (super.getAbsolutePosition()-super.getPositionOffset())*360;
    }
    
}
