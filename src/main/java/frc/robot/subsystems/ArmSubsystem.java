package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    TalonFX armMotor = new TalonFX(10);
    DigitalInput input = new DigitalInput(0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(input);
    DoubleSolenoid extender = new DoubleSolenoid(60,PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid flipper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH,2,3);
    DoubleSolenoid clamper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH, 4, 5);
    public void moveArm(double setPoint, double input){
        if(encoder.getAbsolutePosition() >= setPoint){
            armMotor.set(ControlMode.PercentOutput, input);
        }
    }
    
    public void toggleExtender(){
        extender.toggle();;
    }
    
    public void toggleFlipper(){
        flipper.toggle();
    }
    
    public void toggleClamper(){
        clamper.toggle();
    }

}
