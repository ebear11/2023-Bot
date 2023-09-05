package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autoSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autoSwerve;
public class crappyAuto extends SequentialCommandGroup {
    public crappyAuto(Swerve s_Swerve, ArmSubsystem armSubsystem){
        SequentialCommandGroup dropCube = new SequentialCommandGroup(new MoveToSetpoint(armSubsystem, 1, true),new MoveToSetpoint(armSubsystem, 7, true));
        dropCube.addCommands(new InstantCommand(() -> armSubsystem.extendExtender()), new WaitCommand(1),new InstantCommand(() -> armSubsystem.openClamper()), new InstantCommand(() -> armSubsystem.retractExtender()),new WaitCommand(.5));
        Command autoDrive = new autoSwerve(s_Swerve);
        ParallelCommandGroup driveAuto = new ParallelCommandGroup(autoDrive, new MoveToSetpoint(armSubsystem, 1));


        addCommands(dropCube, driveAuto);
    }
}