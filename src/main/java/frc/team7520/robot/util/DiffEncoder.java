package frc.team7520.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DiffEncoder{

    private DutyCycleEncoder encoder1;
    private DutyCycleEncoder encoder2;

    public DiffEncoder(DutyCycleEncoder encoder1, DutyCycleEncoder encoder2){
        this.encoder1 = encoder1;
        this.encoder2 = encoder2;
    }

    public double get(){
        return encoder1.get() - encoder2.get();
    }

    public double getRate(){
        return encoder1.getFrequency() - encoder2.getFrequency();
    }

    /*
    public void reset(){
        encoder1.reset();
        encoder2.reset();
    }
    public void setDistancePerRotation(double distance){
        encoder1.setDistancePerRotation(distance);
        encoder2.setDistancePerRotation(distance);
    }

    public double getDistancePerRotation(){
        return encoder1.getDistancePerRotation();
    }

    public double getDistance(){
        return encoder1.getDistance() - encoder2.getDistance();
    }
 */
}
