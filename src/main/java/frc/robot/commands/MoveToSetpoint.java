package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class MoveToSetpoint extends CommandBase {
    ArmSubsystem subsystem;
    int setPoint;
    HashMap<Integer, Double[]> posMap;
    public MoveToSetpoint(ArmSubsystem subsystem, int setPoint) {
        this.subsystem = subsystem;
        this.setPoint = setPoint;
        posMap = new HashMap<Integer, Double[]>();
        posMap.put(1, new Double[] {Constants.positionValue.armPos1, Constants.positionValue.flipperPos1});
        posMap.put(2, new Double[] {Constants.positionValue.armPos2, Constants.positionValue.flipperPos2});
        posMap.put(3, new Double[] {Constants.positionValue.armPos3, Constants.positionValue.flipperPos3});
        posMap.put(4, new Double[] {Constants.positionValue.armPos4, Constants.positionValue.flipperPos4});
        posMap.put(5, new Double[] {Constants.positionValue.armPos5, Constants.positionValue.flipperPos5});

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (this.setPoint == 1){
            this.subsystem.moveFlipper(Constants.positionValue.armPos1);
            this.subsystem.moveArm(Constants.positionValue.flipperPos1);
        }
        else if (this.setPoint == 2){
            this.subsystem.moveFlipper(Constants.positionValue.armPos2);
            this.subsystem.moveArm(Constants.positionValue.flipperPos2);
        }
        else if (this.setPoint == 3){
            this.subsystem.moveFlipper(Constants.positionValue.armPos3);
            this.subsystem.moveArm(Constants.positionValue.flipperPos3);
        }
        else if (this.setPoint == 4){
            this.subsystem.moveFlipper(Constants.positionValue.armPos4);
            this.subsystem.moveArm(Constants.positionValue.flipperPos4);
        }
        else if (this.setPoint == 5){
            this.subsystem.moveFlipper(Constants.positionValue.armPos5);
            this.subsystem.moveArm(Constants.positionValue.flipperPos5);
        }
    }
    @Override
    public boolean isFinished() {
        double flipperSetPos = 0;
        double armSetPos = 0;
        double armEncoderValue = 0;
        double flipperEncoderValue = 0;
        armSetPos = posMap.get(setPoint)[0];
        flipperSetPos = posMap.get(setPoint)[1];
        armEncoderValue = this.subsystem.getArmEncoderValue();
        flipperEncoderValue = this.subsystem.getFlipperEncoderValue();
        if ( armEncoderValue > armSetPos+ 2 && armEncoderValue < -armSetPos -2 && flipperEncoderValue > flipperSetPos+1 && armEncoderValue < -armSetPos-1){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
      this.subsystem.stopAllMotors();
    }

}