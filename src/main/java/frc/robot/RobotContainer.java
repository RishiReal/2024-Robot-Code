package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.NoteHandlerCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.ArmNeutralizeAuto;
import frc.robot.commands.AutoCommands.BasicNoteShot;
import frc.robot.commands.AutoCommands.IntakeNoteAuto;
import frc.robot.commands.AutoCommands.NoteAutoShoot;
import frc.robot.commands.AutoCommands.OtherNoteShot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NoteHandler;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

   private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
   private final NoteHandler noteHandler = new NoteHandler();
   private final Climber climber = new Climber();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

   // private final CommandXboxController m_driverController = new CommandXboxController(3);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
          swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.RIGHT_BUMPER),
                () -> driverJoytick.getRawButton(OIConstants.XBX_B),
                () -> driverJoytick.getRawButton(OIConstants.XBX_A)
                )); 
                
        noteHandler.setDefaultCommand(new NoteHandlerCommand(noteHandler, operatorJoystick)); 
        climber.setDefaultCommand(new ClimberCommand(climber, operatorJoystick));

        NamedCommands.registerCommand("ArmNeutral", new ArmNeutralizeAuto(noteHandler));
        NamedCommands.registerCommand("INTAKE", new IntakeNoteAuto(noteHandler));
        NamedCommands.registerCommand("SHOOT", new NoteAutoShoot(noteHandler));
        NamedCommands.registerCommand("BasicShoot", new BasicNoteShot(noteHandler));       
        NamedCommands.registerCommand("OtherShoot", new OtherNoteShot(noteHandler).withTimeout(2.00));           

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser); 
  /* 
        m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(noteHandler.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .whileTrue(noteHandler.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); 
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .whileTrue(noteHandler.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(noteHandler.sysIdDynamic(SysIdRoutine.Direction.kReverse));
*/
        configureButtonBindings();        
    }

    private void configureButtonBindings() {
       /* autoChooser.addOption("4 Note Middle", new PathPlannerAuto("4 Note Middle"));
        autoChooser.addOption("3 Note RIGHT CLOSE (Left on Red)", new PathPlannerAuto("3 Note RIGHT CLOSE (Left on Red)"));
        autoChooser.addOption("3 Note Left LONG - MID", new PathPlannerAuto("3 Note Left LONG - MID"));
        autoChooser.addOption("3 Note Right side FAR (Left on Red)", new PathPlannerAuto("3 Note Right side FAR (Left on Red)"));
        autoChooser.addOption("Middle Start to middle and left (right on red)", new PathPlannerAuto("Middle Start to middle and left (right on red)"));
        autoChooser.addOption("Middle Start to Middle and Right (left on Red)", new PathPlannerAuto("Middle Start to Middle and Right (left on Red)"));
        autoChooser.addOption("Left 2 Note (Right on Red)", new PathPlannerAuto("Left 2 Note (Right on Red)"));
        autoChooser.addOption("Right 2 note (Left on Red)", new PathPlannerAuto("Right 2 note (Left on Red)"));
        autoChooser.addOption("4 Note Left (Right on Red)", new PathPlannerAuto("4 Note Left (Right on Red)"));
        autoChooser.addOption("4 Note Right (Left on Red)", new PathPlannerAuto("4 Note Right (Left on Red)"));
        autoChooser.addOption("5 Note Left", new PathPlannerAuto("5 Note Left"));
        autoChooser.addOption("5 Note Right", new PathPlannerAuto("5 Note Right"));      
        autoChooser.addOption("Left Long Left 4 Note (Right on Red)", new PathPlannerAuto("Left Long Left 4 Note (Right on Red)"));      */
        autoChooser.addOption("Mid 2 Note", new PathPlannerAuto("Mid 2 Note"));      
       // autoChooser.addOption("Actual Mid 2 Note", new PathPlannerAuto("Mid 2 Note Jump Shot"));
        autoChooser.addOption("Drive Out", new PathPlannerAuto("Test Auto"));
    }

    public Command getAutonomousCommand() {     
        //return new ARMDOWN(noteHandler);
       // return (new BasicNoteShot(noteHandler).withTimeout(3.00));
        return autoChooser.getSelected();
        //return (new SimpleAutoDrive(swerveSubsystem));
        //return noteHandler.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        //return noteHandler.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        //return noteHandler.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        //return noteHandler.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        //return (new ShootAndMove(noteHandler, swerveSubsystem));

    }
}
