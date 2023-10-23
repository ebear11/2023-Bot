package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.math.DriveCurve;
import frc.robot.autos.Auto;
import frc.robot.autos.crappyAuto;
import frc.robot.commands.Balance;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.SnapBack;
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
    private final POVButton balance = new POVButton(driver, 0);
    private final POVButton snapBack = new POVButton(driver, 180);
    //private final POVButton PlayMusic = new POVButton(driver, 90);
    // private final POVButton flipperDown = new POVButton(operator, 270);
    private final JoystickButton clampToggle = new JoystickButton(operator, 1);
    private final JoystickButton extendToggle = new JoystickButton(operator, 2);
    private final JoystickButton position1 = new JoystickButton(operator, 7);
    private final JoystickButton position2 = new JoystickButton(operator, 8);
    private final JoystickButton position3 = new JoystickButton(operator, 10);
    private final JoystickButton position4 = new JoystickButton(operator, 12);
    private final JoystickButton position5 = new JoystickButton(operator, 11);
    private final JoystickButton position6 = new JoystickButton(operator, 9);
    private final JoystickButton setLedCone = new JoystickButton(operator, 3);
    private final JoystickButton setLedCube = new JoystickButton(operator, 4);
    private final JoystickButton dropCubeButton = new JoystickButton(operator, 6);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    //private InstantCommand stopMotors = new InstantCommand(() -> armSubsystem.stopAllMotors());
    private SequentialCommandGroup pos1 = new SequentialCommandGroup();
    private SendableChooser<Command> chooser = new SendableChooser<>();
    SequentialCommandGroup dropCube = new SequentialCommandGroup(new MoveToSetpoint(armSubsystem, 1, true),new MoveToSetpoint(armSubsystem, 5, true));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pos1.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()));
        pos1.addCommands(new WaitCommand(.5));
        pos1.addCommands(new MoveToSetpoint(armSubsystem, 1).repeatedly());
        dropCube.addCommands(new WaitCommand(.1),new InstantCommand(() -> armSubsystem.extendExtender()), new WaitCommand(.5),new InstantCommand(() -> armSubsystem.openClamper()), new InstantCommand(() -> armSubsystem.retractExtender()),new WaitCommand(.5), new MoveToSetpoint(armSubsystem, 1, true));
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
        //chooser.setDefaultOption("Simple Auto", new exampleAuto(s_Swerve, armSubsystem));
        //chooser.addOption("Platform Auto", new Auto(s_Swerve, armSubsystem));
        //SmartDashboard.putData(chooser);

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
        // liftUp
        //     .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(.5)))
        //     .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        // liftDown
        //     .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(-.5)))
        //     .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        // flipperUp
        //     .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(.25)))
        //     .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        // flipperDown
        //     .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(-.25)))
        //     .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        balance.onTrue(new Balance(s_Swerve));
        snapBack.whileTrue(new SnapBack(s_Swerve));
        extendToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleExtender()));
        clampToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleClamper()));
        position1
        .whileTrue(pos1);
        position2.whileTrue(new MoveToSetpoint(armSubsystem, 2));
        position3.whileTrue(new MoveToSetpoint(armSubsystem, 3));
        position4.whileTrue(new MoveToSetpoint(armSubsystem, 4));
        position5.whileTrue(new MoveToSetpoint(armSubsystem, 5));
        position6.whileTrue(new MoveToSetpoint(armSubsystem, 6));
        setLedCone.onTrue(new InstantCommand(() -> armSubsystem.setLedCone()));
        setLedCube.onTrue(new InstantCommand(() -> armSubsystem.setLedCube()));
        dropCubeButton.onTrue(dropCube);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auto(s_Swerve, armSubsystem);
        //return chooser.getSelected();
        //return new exampleAuto(s_Swerve, armSubsystem);
        //return new crappyAuto(s_Swerve, armSubsystem);
        //return new WaitCommand(1);
    }
}
