package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ToggleArm extends CommandBase {    
    private final ArmSubsystem armSubsystem;
    Joystick stick = new Joystick(1);
    public ToggleArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.moveArm(Constants.armSetpoint, stick.getRawAxis(0));       
    }
}