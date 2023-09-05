package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class autoSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    Timer timer;
    public autoSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        timer = new Timer();
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -.47;
        double strafeVal = 0;
        double rotationVal = 0;
        if (timer.get() > Constants.crappyAutoTime){
            translationVal = 0;
        }
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                true, 
                true);
        /* Drive */
        
    }
}