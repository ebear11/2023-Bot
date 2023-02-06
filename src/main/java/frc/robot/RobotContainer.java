package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.Auto;
import frc.robot.commands.ArmDown;
import frc.robot.commands.PIDRamp;
import frc.robot.commands.ArmUp;
import frc.robot.commands.ToggleClamper;
import frc.robot.commands.ToggleExtender;
import frc.robot.commands.ToggleFlipper;
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
    private final JoystickButton tiltToggle = new JoystickButton(operator, 5);
    private final JoystickButton clampToggle = new JoystickButton(operator, 2);
    private final JoystickButton extendToggle = new JoystickButton(operator, 3);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private Command PIDRamp = new PIDRamp(s_Swerve).repeatedly();
    private Command ArmDown = new ArmDown(armSubsystem);
    private Command ArmUp = new ArmUp(armSubsystem);
    private Command extend = new ToggleExtender(armSubsystem);
    private Command flip = new ToggleFlipper(armSubsystem);
    private Command clamp = new ToggleClamper(armSubsystem);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> .75*(Math.pow(-driver.getRawAxis(translationAxis),3))+ (1 - .75)* -driver.getRawAxis(translationAxis), 
                () -> .75*(Math.pow(-driver.getRawAxis(strafeAxis),3))+ (1 - .75)* -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

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
        liftUp.onTrue(ArmUp);
        liftDown.onTrue(ArmDown);
        extendToggle.onTrue(extend);
        clampToggle.onTrue(clamp);
        tiltToggle.onTrue(flip);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new Auto(s_Swerve);
    }
}
