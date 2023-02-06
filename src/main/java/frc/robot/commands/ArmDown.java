package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmDown extends CommandBase {    
    private final ArmSubsystem armSubsystem;
    public ArmDown(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
    armSubsystem.moveArm(10000, -.5);       
    }
}