package frc.robot.libs;

import edu.wpi.first.wpilibj.XboxController;

public class XboxCotroller extends XboxController{
    public XboxCotroller(int port){
        super(port);
    }
    double deadband = .07;
    @Override
    public double getLeftX(){
        if(Math.abs(super.getLeftX()) > deadband){
            return Math.pow(super.getLeftX(),3);
        }else{
            return 0;
        }
    }

    @Override
    public double getRightX(){
        if(Math.abs(super.getRightX()) > deadband){
            return Math.pow(super.getRightX(),3);
        }else{
            return 0;
        }
    }

    @Override
    public double getLeftY(){
        if(Math.abs(super.getLeftY()) > deadband){
            return Math.pow(super.getLeftY(),3);
        }else{
            return 0;
        }
    }

    @Override
    public double getRightY(){
        if(Math.abs(super.getRightY()) > deadband){
            return Math.pow(super.getRightY(),3);
        }else{
            return 0;
        }
    }
}
