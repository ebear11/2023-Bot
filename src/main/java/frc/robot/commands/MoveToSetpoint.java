package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveToSetpoint extends CommandBase {
    ArmSubsystem subsystem;
    int setPoint;
    HashMap<Integer, double[]> posMap;
    public MoveToSetpoint(ArmSubsystem subsystem, int setPoint) {
        this.subsystem = subsystem;
        this.setPoint = setPoint;
        posMap = new HashMap<Integer, double[]>();
        posMap.put(1, new double[] {Constants.PositionValue.armPos1, Constants.PositionValue.flipperPos1});
        posMap.put(2, new double[] {Constants.PositionValue.armPos2, Constants.PositionValue.flipperPos2});
        posMap.put(3, new double[] {Constants.PositionValue.armPos3, Constants.PositionValue.flipperPos3});
        posMap.put(4, new double[] {Constants.PositionValue.armPos4, Constants.PositionValue.flipperPos4});
        posMap.put(5, new double[] {Constants.PositionValue.armPos5, Constants.PositionValue.flipperPos5});

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.retractExtender();
    }
    @Override
    public boolean isFinished(){
        return subsystem.atSetpoint();
    }
}