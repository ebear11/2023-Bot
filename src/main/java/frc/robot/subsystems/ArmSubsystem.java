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
    CANSparkMax flipperMotor = new CANSparkMax(32, MotorType.kBrushless);

    DigitalInput input = new DigitalInput(0);
    DigitalInput input2 = new DigitalInput(1);
    DutyCycleEncoder encoder = new DutyCycleEncoder(input);
    DutyCycleEncoder encoderFlipper = new DutyCycleEncoder(input2);
    DoubleSolenoid extender = new DoubleSolenoid(60,PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid flipper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH,2,3);
    DoubleSolenoid clamper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH, 4, 5);
    public void moveArm(double setPoint, double input){
        System.out.println("Encoder val: " + encoder.getAbsolutePosition());
        if(encoder.getAbsolutePosition() >= setPoint){
            armMotor.set(input);
        }
        else {
            armMotor.stopMotor();}
    }

    public void moveArm(double setPoint){
        if (encoder.getAbsolutePosition() > setPoint + 2){
            armMotor.set(.35);
        }
        else if (encoder.getAbsolutePosition() < setPoint -2 ){
            armMotor.set(-.35);
        }
        else {
            armMotor.stopMotor();
        }
    }
    public void moveFlipperMan(double speed){
        flipperMotor.set(speed);
    }
    public void moveFlipper(double setPoint){
        if (encoderFlipper.getAbsolutePosition() > setPoint + 1){
            flipperMotor.set(.15);
        }
        else if (encoderFlipper.getAbsolutePosition() < setPoint -1){
            flipperMotor.set(-.15);
        }
        else {
            flipperMotor.stopMotor();
        }
    }
    public void stopAllMotors(){
        flipperMotor.stopMotor();
        armMotor.stopMotor();
    }
    
    public void toggleExtender(){
        extender.toggle();
    }
    
    public void toggleClamper(){
        clamper.toggle();
    }

    public void setPuller(double speed){
        puller.set(speed);
    }
    public double getArmEncoderValue(){
        return encoder.getAbsolutePosition();
    }
    public double getFlipperEncoderValue(){
        return encoderFlipper.getAbsolutePosition();
    }
}
