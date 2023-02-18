package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax puller = new CANSparkMax(31, MotorType.kBrushless);
    CANSparkMax armMotor = new CANSparkMax(30, MotorType.kBrushless);
    TalonFX flipperMotor = new TalonFX(32);
    PIDController flipperController = new PIDController(Constants.flipperP,0,0);
    PIDController armController = new PIDController(Constants.armP, 0, 0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(0));
    DutyCycleEncoder encoderFlipper = new DutyCycleEncoder(new DigitalInput(1));
    DoubleSolenoid extender = new DoubleSolenoid(60,PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid flipper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH,2,3);
    DoubleSolenoid clamper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH, 4, 5);
    public void moveArmMan(double input){
        armMotor.set(input);
    }
    
    public void moveArm(double setPoint){
        armController.setTolerance(Constants.armTol);
        armController.setSetpoint(setPoint);
        if (!armController.atSetpoint()){
        armMotor.set(armController.calculate(encoder.getAbsolutePosition(), setPoint));
        }
        else{
            armMotor.stopMotor();
        }
    }
    public void moveFlipperMan(double speed){
        flipperMotor.set(ControlMode.PercentOutput, speed);
    }
    public void moveFlipper(double setPoint){
        flipperController.setTolerance(Constants.flipperTol);
        flipperController.setSetpoint(setPoint);
        if (!flipperController.atSetpoint()) {
        flipperMotor.set(ControlMode.PercentOutput, flipperController.calculate(encoderFlipper.getAbsolutePosition(), setPoint));
        }
        else {
            flipperMotor.set(ControlMode.PercentOutput, 0);
        }
    }
    public void stopAllMotors(){
        flipperMotor.set(ControlMode.PercentOutput, 0);
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
    public void retractExtender(){
        extender.set(Value.kReverse);;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Value", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Flipper Encoder Value", encoderFlipper.getAbsolutePosition());

    }
}
