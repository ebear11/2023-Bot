package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SnapBack extends CommandBase {
    Swerve swerve;
    double rotVal;
    Pigeon2 pigeon = new Pigeon2(Constants.Swerve.pigeonID);
    PIDController turnController = new PIDController(Constants.Swerve.angleKP, 0, 0);
    public SnapBack(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);

    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute() {
        turnController.enableContinuousInput(0, 360);
        rotVal = MathUtil.clamp(turnController.calculate(pigeon.getYaw(), 0),-5,5);
        swerve.drive(new Translation2d(0,0), rotVal, true, false);
    }
    // @Override
    // public boolean isFinished(){
    //     if (pigeon.getYaw() > 1 && pigeon.getYaw() < -1){
    //         return true;
    //     }
    //     return false;
    // }

    
}
