package frc.robot;

import java.util.function.BooleanSupplier;

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
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.PIDRamp;
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
    private final JoystickButton PID = new JoystickButton(driver, XboxController.Button.kX.value);
    private final POVButton liftUp = new POVButton(operator, 0);
    private final POVButton liftDown = new POVButton(operator, 180);
    private final POVButton flipperUp = new POVButton(operator, 90);
    private final POVButton flipperDown = new POVButton(operator, 270);
    private final JoystickButton clampToggle = new JoystickButton(operator, 2);
    private final JoystickButton extendToggle = new JoystickButton(operator, 3);
    private final JoystickButton pull = new JoystickButton(operator, 6);
    private final JoystickButton push = new JoystickButton(operator, 4);
    private final JoystickButton position1 = new JoystickButton(operator, 7);
    private final JoystickButton position2 = new JoystickButton(operator, 8);
    private final JoystickButton position3 = new JoystickButton(operator, 10);
    private final JoystickButton position4 = new JoystickButton(operator, 12);
    private final JoystickButton position5 = new JoystickButton(operator, 11);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private Command PIDRamp = new PIDRamp(s_Swerve).repeatedly();
    private InstantCommand stopMotors = new InstantCommand(() -> armSubsystem.stopAllMotors());
    private SequentialCommandGroup grabPOS = new SequentialCommandGroup();
    private SequentialCommandGroup dropTop = new SequentialCommandGroup();
    private SequentialCommandGroup dropMid = new SequentialCommandGroup();
    private SequentialCommandGroup idle = new SequentialCommandGroup();
    private SequentialCommandGroup ground = new SequentialCommandGroup();
    ConditionalCommand idleDefault = new ConditionalCommand(idle, new WaitCommand(1), () -> !armSubsystem.getStop());

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        grabPOS.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()));
        grabPOS.addCommands(new WaitCommand(1), new MoveToSetpoint(armSubsystem, 1));
        grabPOS.addCommands(new InstantCommand(() -> armSubsystem.openClamper()), new WaitCommand(.1));
        grabPOS.addCommands(new InstantCommand(() -> armSubsystem.setPuller(1)), new MoveToSetpoint(armSubsystem, 2));
        grabPOS.addCommands(new InstantCommand(() -> armSubsystem.closeClamper()), new WaitCommand(.1), new InstantCommand(() -> armSubsystem.setPuller(0)));

        dropTop.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()), new MoveToSetpoint(armSubsystem, 2));
        dropTop.addCommands(new InstantCommand(() -> armSubsystem.toggleExtender()), new InstantCommand(() -> armSubsystem.setPuller(-1)));
        dropTop.addCommands(new WaitCommand(.1), new InstantCommand(() -> armSubsystem.setPuller(0)));

        dropMid.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()));
        dropMid.addCommands(new MoveToSetpoint(armSubsystem, 3));
        dropMid.addCommands(new InstantCommand(() -> armSubsystem.setPuller(-1)), new WaitCommand(.1), new InstantCommand(() -> armSubsystem.setPuller(0)));

        ground.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()), new MoveToSetpoint(armSubsystem, 4));
        ground.addCommands(new InstantCommand(() -> armSubsystem.toggleExtender()), new InstantCommand(() -> armSubsystem.setPuller(-1)));
        ground.addCommands(new WaitCommand(.1), new InstantCommand(() -> armSubsystem.setPuller(0)));
        idle.addCommands(new InstantCommand(() -> armSubsystem.retractExtender()), new MoveToSetpoint(armSubsystem, 5));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> DriveCurve.applyDriveCurve(-driver.getRawAxis(translationAxis)), 
                () -> DriveCurve.applyDriveCurve(-driver.getRawAxis(strafeAxis)), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        armSubsystem.setDefaultCommand(idleDefault);
//        armSubsystem.setDefaultCommand(idle);
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
        PID.whileTrue(PIDRamp);
        liftUp
            .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(1)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        liftDown
            .onTrue(new InstantCommand(() -> armSubsystem.moveArmMan(-1)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveArmMan(0)));
        pull
            .onTrue(new InstantCommand(() -> armSubsystem.setPuller(1)))
            .onFalse(new InstantCommand(() -> armSubsystem.setPuller(0)));
        push
            .onTrue(new InstantCommand(() -> armSubsystem.setPuller(-1)))
            .onFalse(new InstantCommand(() -> armSubsystem.setPuller(0)));
        flipperUp
            .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(.25)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        flipperDown
            .onTrue(new InstantCommand(() -> armSubsystem.moveFlipperMan(-.25)))
            .onFalse(new InstantCommand(() -> armSubsystem.moveFlipperMan(0)));
        extendToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleExtender()));
        clampToggle.onTrue(new InstantCommand(() -> armSubsystem.toggleClamper()));
        position1.whileTrue(new MoveToSetpoint(armSubsystem, 1).repeatedly());
        position2.whileTrue(grabPOS);
        position3.whileTrue(new MoveToSetpoint(armSubsystem, 3).repeatedly());
        position4.whileTrue(new MoveToSetpoint(armSubsystem, 4).repeatedly());
        position5.whileTrue(new MoveToSetpoint(armSubsystem, 5).repeatedly());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new Auto(s_Swerve, armSubsystem);
    }
}
