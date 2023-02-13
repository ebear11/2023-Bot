package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax puller = new CANSparkMax(31, MotorType.kBrushless);
    CANSparkMax armMotor = new CANSparkMax(30, MotorType.kBrushless);
    DigitalInput input = new DigitalInput(0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(input);
    DoubleSolenoid extender = new DoubleSolenoid(60,PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid flipper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH,2,3);
    DoubleSolenoid clamper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH, 4, 5);
    public void moveArm(double setPoint, double input){
        System.out.println("Encoder val: " + encoder.getAbsolutePosition());
        if(encoder.getAbsolutePosition() >= setPoint){
            armMotor.set(input);
        }
        else {
            armMotor.set(0);}
    }
    
    public void toggleExtender(){
        extender.toggle();
    }
    
    public void toggleFlipper(){
        flipper.toggle();
    }
    
    public void toggleClamper(){
        clamper.toggle();
    }
    public void setPuller(double speed){
        puller.set(speed);
    }
    

}
