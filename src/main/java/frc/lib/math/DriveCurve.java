package frc.lib.math;

import frc.robot.Constants;

public class DriveCurve {
    public static double applyDriveCurve(double value){
       return Constants.driveCurveConstant*(Math.pow(-value,3))+(1 - Constants.driveCurveConstant)*-value;
    }
}
