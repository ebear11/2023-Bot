package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.math.DriveCurve;
import frc.robot.autos.Auto;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.MoveToSetpoint;
//import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final POVButton liftUp = new POVButton(operator, 0);
    private final POVButton liftDown = new POVButton(operator, 180);
    private final POVButton flipperUp = new POVButton(operator, 90);
    private final POVButton flipperDown = new POVButton(operator, 270);
    private final JoystickButton clampToggle = new JoystickButton(operator, 1);
    private final JoystickButton extendToggle = new JoystickButton(operator, 2);
    private final JoystickButton position1 = new JoystickButton(operator, 7);
    private final JoystickButton position2 = new JoystickButton(operator, 8);
    private final JoystickButton position3 = new JoystickButton(operator, 10);
    private final JoystickButton position4 = new JoystickButton(operator, 12);
    private final JoystickButton position5 = new JoystickButton(operator, 11);
    private final JoystickButton stopArm = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton startArm = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    //private InstantCommand stopMotors = new InstantCommand(() -> armSubsystem.stopAllMotors());
    private SequentialCommandGroup grabPOS = new SequentialCommandGroup();
    private SequentialCommandGroup dropTop = new SequentialCommandGroup();
    private SequentialCommandGroup dropMid = new SequentialCommandGroup();
    private SequentialCommandGroup idle = new SequentialCommandGroup();
    private SequentialCommandGroup ground = new SequentialCommandGroup();
    private SequentialCommandGroup pos1 = new SequentialCommandGroup();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pos1.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()));
        pos1.addCommands(new WaitCommand(.5));
        pos1.addCommands(new MoveToSetpoint(armSubsystem, 1).repeatedly());
        idle.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()), new WaitCommand(2), new MoveToSetpoint(armSubsystem, 5));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> DriveCurve.applyDriveCurve(-driver.getRawAxis(translationAxis)), 
                () -> DriveCurve.applyDriveCurve(-driver.getRawAxis(strafeAxis)), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
//        armSubsystem.setDefaultCommand(idleDefault);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        liftUp
            .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(.5)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        liftDown
            .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(-.5)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        flipperUp
            .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(.25)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        flipperDown
            .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(-.25)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        extendToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleExtender()));
        clampToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleClamper()));
        position1
        .whileTrue(pos1);
        position2.whileTrue(new MoveToSetpoint(armSubsystem, 2));
        position3.whileTrue(new MoveToSetpoint(armSubsystem, 3));
        position4.whileTrue(new MoveToSetpoint(armSubsystem, 4));
        position5.whileTrue(new MoveToSetpoint(armSubsystem, 5));
        stopArm.onTrue(new InstantCommand(() -> armSubsystem.setStop(true)));
        startArm.onTrue(new InstantCommand(() -> armSubsystem.setStop(false)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
        //return new Auto(s_Swerve, armSubsystem);
        //return new WaitCommand(1);
    }
}
