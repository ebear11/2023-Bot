package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
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
    TalonFX armMotor = new TalonFX(31);
    TalonFX armMotor2 = new TalonFX(33);
    CANSparkMax flipperMotor = new CANSparkMax(32, MotorType.kBrushless);
    PIDController flipperController = new PIDController(Constants.flipperP,0,0);
    PIDController armController = new PIDController(Constants.armP, 0, 0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(1));
    DutyCycleEncoder encoderFlipper = new DutyCycleEncoder(new DigitalInput(0));
    DoubleSolenoid extender = new DoubleSolenoid(60,PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid clamper = new DoubleSolenoid(60, PneumaticsModuleType.REVPH,2,3);
    boolean stopArm = false;
    public ArmSubsystem() {
        extender.set(Value.kForward);
        clamper.set(Value.kForward);
    }
    public void moveArmMan(double input){
        armMotor.set(ControlMode.PercentOutput, -input);
        armMotor2.set(ControlMode.PercentOutput, input);
    }
    
    public void moveArm(double setPoint){
        armController.setTolerance(Constants.armTol);
        armController.setSetpoint(setPoint);
        if (!armController.atSetpoint()){
            double speed = armController.calculate(encoder.getAbsolutePosition(), setPoint);
            speed = MathUtil.clamp(speed, -.25, .25);
            armMotor.set(ControlMode.PercentOutput, -speed);
            armMotor2.set(ControlMode.PercentOutput, speed);
        }
        else{
            armMotor.set(ControlMode.PercentOutput, 0);
            armMotor2.set(ControlMode.PercentOutput, 0);
        }
    }
    public void moveFlipperMan(double speed){
        flipperMotor.set(speed);
    }
    public void moveFlipper(double setPoint){
        flipperController.setTolerance(Constants.flipperTol);
        flipperController.setSetpoint(setPoint);
        double speed = flipperController.calculate(encoderFlipper.getAbsolutePosition(), setPoint);
        speed = MathUtil.clamp(speed, -.25, .25);
        if (!flipperController.atSetpoint()) {
            flipperMotor.set(-speed);
        }
        else {
            flipperMotor.set(0);
        }
    }
    public boolean atSetpoint(){
        return flipperController.atSetpoint() && armController.atSetpoint();
    }
    public void stopAllMotors(){
        flipperMotor.set(0);
        armMotor.set(ControlMode.PercentOutput, 0);
        armMotor2.follow(armMotor);
    }
    
    public void toggleExtender(){
        extender.toggle();
    }
    
    public void toggleClamper(){
        clamper.toggle();
    }

    public void retractExtender(){
        extender.set(Value.kForward);
    }
    public void extendExtender(){
        extender.set(Value.kReverse);
    }
    public void openClamper(){
        clamper.set(Value.kReverse);
    }
    public void closeClamper(){
        clamper.set(Value.kForward);
    }
    public boolean getStop(){
        return stopArm;
    }
    public void setStop(boolean value){
        stopArm = value;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Value", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Flipper Encoder Value", encoderFlipper.getAbsolutePosition());

    }
}