package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
    Swerve swerve;
    double translationVal;
    Pigeon2 pigeon = new Pigeon2(Constants.Swerve.pigeonID);
    PIDController balanceController = new PIDController(Constants.balanceP, 0, 0);
    public Balance(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);

    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute() {
        translationVal = MathUtil.clamp(balanceController.calculate(pigeon.getPitch(), 0),-.15,.15);
        swerve.drive(new Translation2d(translationVal,0).times(Constants.Swerve.maxSpeed), 0, true, false);
    }
    @Override
    public boolean isFinished(){
        if (pigeon.getPitch() > 2 && pigeon.getPitch() < -2){
            return true;
        }
        return false;
    }

    
}
