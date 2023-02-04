package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class FlipUp extends CommandBase {    
    private final ArmSubsystem armSubsystem;
    public FlipUp(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
    armSubsystem.flipUp();       
    }
}