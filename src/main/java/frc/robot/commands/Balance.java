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
    boolean stop = true;
    public Balance(Swerve swerve){
        this.balanceController.setTolerance(0.25);
        this.swerve = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        stop = false;
    }
    @Override
    public void execute() {
        //translationVal = MathUtil.clamp(balanceController.calculate(pigeon.getPitch(), 0),-.1,.1);
        System.out.println("Running Balance");
        translationVal = 0;
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        // if (pigeon.getPitch() < -0.2){
        //     translationVal = .11;
        // }
        if (pigeon.getPitch() > 0.1){
            translationVal = -.11;
        }

        if(xyz[0] > 11.5){
            translationVal = 0;
            stop = true;
        }
        else if(xyz[0] > 7.5){
            translationVal = .09;
        }
        swerve.drive(new Translation2d(translationVal,0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }
    @Override
    public boolean isFinished(){
         return stop;
     }
    @Override
    public void end(boolean interrupted){

        swerve.drive(new Translation2d(0,0).times(Constants.Swerve.maxSpeed), 0, true, false);

    }

    
}
