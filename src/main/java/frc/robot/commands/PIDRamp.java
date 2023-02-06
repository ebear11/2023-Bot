package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PIDRamp extends CommandBase {    
    private Swerve s_Swerve;    
    private PIDController pidController;
    public PIDRamp(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        this.pidController = new PIDController(.2,0,.5);
        pidController.setTolerance(3.0);
        pidController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = pidController.calculate(s_Swerve.gyro.getRoll(), 0);
        System.out.println("Error: " + pidController.getPositionError());
        System.out.println("Roll: " + s_Swerve.gyro.getRoll());
        System.out.println("motor output: " + translationVal);
        System.out.println("Tolerance " + pidController.getPositionTolerance());
        translationVal /= 4.5;
        /* Drive */
        boolean atSet = pidController.atSetpoint();
        if (!atSet){
            s_Swerve.drive(
            new Translation2d(translationVal, 0), 
            0, 
            false, 
            true
        );
        }
        
    }
}