package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PIDRamp extends CommandBase {    
    private Swerve s_Swerve;    
    private PIDController pidController;
    public PIDRamp(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        this.pidController = new PIDController(.1,0,.5);
        pidController.setTolerance(1);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = pidController.calculate(s_Swerve.gyro.getPitch(), 0);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            false, 
            true
        );
    }
}