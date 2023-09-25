package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.commands.Balance;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.autoSwerve;
import frc.robot.subsystems.ArmSubsystem;

public class Auto extends SequentialCommandGroup{
    public Auto(Swerve s_Swerve, ArmSubsystem armSubsystem){
        SequentialCommandGroup dropCube = new SequentialCommandGroup(new MoveToSetpoint(armSubsystem, 1, true),new MoveToSetpoint(armSubsystem, 7, true));
        dropCube.addCommands(new InstantCommand(() -> armSubsystem.extendExtender()), new WaitCommand(1),new InstantCommand(() -> armSubsystem.openClamper()), new InstantCommand(() -> armSubsystem.retractExtender()),new WaitCommand(.5), new MoveToSetpoint(armSubsystem, 1));
       // Command autoDrive = new autoSwerve(s_Swerve);
        PathPlannerTrajectory Path = PathPlanner.loadPath("HardPath", new PathConstraints(4, 3));

    //    / ParallelCommandGroup driveAuto = new ParallelCommandGroup(autoDrive, new MoveToSetpoint(armSubsystem, 1));

        PPSwerveControllerCommand autoController = new PPSwerveControllerCommand(
            Path, 
            s_Swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            s_Swerve::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve // Requires this drive subsystem
        );
        Command balanceBot = new Balance(s_Swerve);
        addCommands(dropCube, new InstantCommand(() -> s_Swerve.resetOdometry(Path.getInitialHolonomicPose())), autoController, balanceBot);
    }
    
}
