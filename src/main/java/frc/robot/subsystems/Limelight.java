package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    Pose2d getPose(){
        double[] botpose = new double[6];
        Pose2d pose = new Pose2d();
        if(DriverStation.getAlliance() == Alliance.Blue){
            botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            pose = new Pose2d();
        }
        else if(DriverStation.getAlliance() == Alliance.Red){
            botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpibred").getDoubleArray(new double[6]);
            pose = new Pose2d();
        }
        return pose;
    }
}