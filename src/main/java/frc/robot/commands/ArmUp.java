package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmUp extends CommandBase {    
    private final ArmSubsystem armSubsystem;
    public ArmUp(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.armUp(Constants.armSetpoint);       
    }
}