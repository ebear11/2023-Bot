package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.MoveToSetpoint;
//import frc.robot.commands.PIDRamp;
import frc.robot.subsystems.ArmSubsystem;
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
    public Auto(Swerve s_Swerve, ArmSubsystem armSubsystem){
        HashMap<String, Command> eventMap = new HashMap<>();
        SequentialCommandGroup grabCube = new SequentialCommandGroup(new InstantCommand(() -> armSubsystem.openClamper()));
        grabCube.addCommands(new MoveToSetpoint(armSubsystem, 1));
        grabCube.addCommands(new WaitCommand(.5));
        grabCube.addCommands(new InstantCommand(()-> armSubsystem.toggleClamper()));
        grabCube.addCommands(new MoveToSetpoint(armSubsystem, 3));

        // wait command placeholder
        eventMap.put("drop cube", new InstantCommand(() -> armSubsystem.openClamper()));
        eventMap.put("grab cube", grabCube);
        eventMap.put("drop cube", new InstantCommand(() -> armSubsystem.openClamper()));

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