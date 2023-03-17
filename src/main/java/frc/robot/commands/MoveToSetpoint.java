package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveToSetpoint extends CommandBase {
    ArmSubsystem subsystem;
    int setPoint;
    HashMap<Integer, double[]> posMap;
    Timer timer = new Timer();
    public MoveToSetpoint(ArmSubsystem subsystem, int setPoint) {
        this.subsystem = subsystem;
        this.setPoint = setPoint;
        posMap = new HashMap<Integer, double[]>();
        posMap.put(1, new double[] {Constants.PositionValue.armPos1, Constants.PositionValue.flipperPos1});
        posMap.put(2, new double[] {Constants.PositionValue.armPos2, Constants.PositionValue.flipperPos2});
        posMap.put(3, new double[] {Constants.PositionValue.armPos3, Constants.PositionValue.flipperPos3});
        posMap.put(4, new double[] {Constants.PositionValue.armPos4, Constants.PositionValue.flipperPos4});
        posMap.put(5, new double[] {Constants.PositionValue.armPos5, Constants.PositionValue.flipperPos5});
        posMap.put(6, new double[] {Constants.PositionValue.armPos6, Constants.PositionValue.flipperPos6});
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        if (setPoint == 4){
            if (timer.get() > 1.195){
                subsystem.moveFlipper(posMap.get(setPoint)[1]);
            }
        }
        else if (setPoint == 1){
            if (timer.get() > .5){
                subsystem.moveFlipper(posMap.get(setPoint)[1]);
            } 
        }
        else if (setPoint == 5){
            if (timer.get() > 1.3){
                subsystem.moveFlipper(posMap.get(setPoint)[1]);
            }
        }
        else if (setPoint == 2){
            if (timer.get() > .67){
                subsystem.moveFlipper(posMap.get(setPoint)[1]);
            }
        }
        else {
            subsystem.moveFlipper(posMap.get(setPoint)[1]);        
        }
        if (setPoint == 1){
            if (timer.get() > .5){
                subsystem.moveArm(posMap.get(setPoint)[0]);
            }
        }
        else {
            subsystem.moveArm(posMap.get(setPoint)[0]);
        }

    }
    @Override
    public boolean isFinished(){
        return subsystem.atSetpoint();
    }
    @Override
    public void end(boolean interrupted){
        subsystem.stopAllMotors();
    }
}