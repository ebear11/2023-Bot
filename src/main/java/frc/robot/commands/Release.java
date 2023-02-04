package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Release extends CommandBase {    
    private final ArmSubsystem armSubsystem;
    public Release(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
    armSubsystem.release();       
    }
}