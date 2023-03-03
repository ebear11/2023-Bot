package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    NetworkTableInstance inst;
    NetworkTable limelightTable;

    public Limelight(){
        this.inst = NetworkTableInstance.getDefault();
        this.limelightTable = inst.getTable("limelight");
    }
    NetworkTableEntry getEntry(String value){
        return this.limelightTable.getEntry(value);
    }
    public boolean hasTarget(){
        return getEntry("tv").getBoolean(false);
    }
    public double getXOffset(){
        return getEntry("tx").getDouble(0.0);
    }
    public double getYOffset(){
        return getEntry("ty").getDouble(0.0);
    }
    public double getArea(){
        return getEntry("ta").getDouble(0.0);
    }
}
