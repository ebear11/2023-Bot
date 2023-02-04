package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.PIDRamp;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
public class Auto extends SequentialCommandGroup {
    public Auto(Swerve s_Swerve){
        HashMap<String, Command> eventMap = new HashMap<>();
        // wait command placeholder
        eventMap.put("drop cube", new WaitCommand(5));
        eventMap.put("grab cube", new WaitCommand(2));
        eventMap.put("drop cube", new WaitCommand(5));
        eventMap.put("balance", new PIDRamp(s_Swerve).repeatedly());

        PathPlannerTrajectory path = PathPlanner.loadPath("Path", new PathConstraints(4, 3));
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                path,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
            FollowPathWithEvents command = new FollowPathWithEvents(swerveControllerCommand,path.getMarkers(),eventMap);
            addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(path.getInitialPose())),
                command
            );
    }
}