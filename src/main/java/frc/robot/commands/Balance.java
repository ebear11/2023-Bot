package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
    Swerve swerve;
    double translationVal;
    Pigeon2 pigeon = new Pigeon2(Constants.Swerve.pigeonID);
    PIDController balanceController = new PIDController(Constants.balanceP, 0, 0);
    Timer timer = new Timer();
    public Balance(Swerve swerve){
        this.balanceController.setTolerance(0.25);
        this.swerve = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        //translationVal = MathUtil.clamp(balanceController.calculate(pigeon.getPitch(), 0),-.1,.1);
        System.out.println("Running Balance");
        translationVal = 0;
        if (pigeon.getPitch() < -0.5){
            translationVal = .15;
        }
        else if (pigeon.getPitch() > 0.5){
            translationVal = -.15;
        }
        swerve.drive(new Translation2d(translationVal,0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }
    // @Override
    // public boolean isFinished(){
    //     if (pigeon.getPitch() > 10 && pigeon.getPitch() < -10){
    //         return false;
    //     }
    //     return true;
    // }

    
}
