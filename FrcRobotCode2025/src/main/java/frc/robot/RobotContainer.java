// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.ArmCmds.AmpAnglePIDCmd;
//import frc.robot.commands.ArmCmds.ArmDrive;
//import frc.robot.commands.ArmCmds.ArmDrivePIDCmd;
//import frc.robot.commands.ArmCmds.RestAnglePIDCmd;
//import frc.robot.commands.ArmCmds.ShooterAnglePIDCmd;
import frc.robot.commands.FlyWheelCmds.FlyWheelAmp;
import frc.robot.commands.FlyWheelCmds.FlyWheelCmd;
import frc.robot.commands.IntakeCmds.IntakeCmd;
import frc.robot.commands.IntakeCmds.IntakeInCmd;
import frc.robot.commands.VisionCmds.DriveAimAtTarget;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  
  //private final ArmSubsystem arm = new ArmSubsystem();
  private final CoralSubsystem trapArm = new CoralSubsystem(); 
  private final Intake intake = new Intake();
  private final FlyWheel flywheel = new FlyWheel();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandJoystick m_driverController = new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_systemController = new CommandXboxController(Constants.OperatorConstants.kSystemControllerPort);

  private SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("IntakeInCmd", new IntakeInCmd(intake));
    NamedCommands.registerCommand("FlyWheelCmd", new FlyWheelCmd(flywheel, 1800));
    //NamedCommands.registerCommand("ShooterAnglePIDCmd", new ShooterAnglePIDCmd(arm));
    //NamedCommands.registerCommand("AmpAnglePIDCmd", new AmpAnglePIDCmd(arm));
    //NamedCommands.registerCommand("RestAnglePIDCmd", new RestAnglePIDCmd(arm));
    // Configure the trigger bindings
    configureBindings();

    initializeChooser();
    



      //Aplica as DeadBands e Inverte os controles por que os joysticks
      //estão positivos para trás e para direita
      //enquanto o movimento do robô está positivo
      //para frente e para esquerda
      //joystick esquerdo controla movimentação 
      //joystick direito controla a velocidade angular do robô
    Command baseDriveCommand = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-m_driverController.getY() * Math.max(-m_driverController.getRawAxis(3) + 1, 0.3), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getX()* Math.max(-m_driverController.getRawAxis(3)+ 1, 0.3), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -m_driverController.getZ() * Math.max(-m_driverController.getRawAxis(3)+ 1, 0.565));

    

      drivebase.setDefaultCommand(baseDriveCommand);
      intake.setDefaultCommand(new IntakeCmd(intake, () -> m_systemController.getRightTriggerAxis(),
      () -> m_systemController.getLeftTriggerAxis()));
      }

    public static void setRightRumbleDriver(double rumble){
      m_driverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

     m_driverController.button(2).toggleOnTrue(new DriveAimAtTarget(drivebase,
      () -> MathUtil.applyDeadband(-m_driverController.getY() * Math.max(-m_driverController.getRawAxis(3)+ 1, 0.3), Constants.OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getX() * Math.max(-m_driverController.getRawAxis(3)+ 1, 0.3), Constants.OperatorConstants.LEFT_Y_DEADBAND)));
    
    m_driverController.button(3).onTrue(Commands.runOnce(drivebase::zeroGyro));
    m_driverController.button(4).onTrue(Commands.runOnce(drivebase::resetIMU));

    m_driverController.button(8).toggleOnTrue(new FlyWheelCmd(flywheel, 1800));
    m_systemController.leftBumper().toggleOnTrue(new FlyWheelAmp(flywheel));
    /*m_driverController.button(7).onTrue(trapArm.setArmGoalCommand(50, 10));
    m_driverController.button(9).onTrue(trapArm.setArmGoalCommand(40, 10));
    m_driverController.button(10).onTrue(trapArm.setArmGoalCommand(30, 10));
    m_driverController.button(11).onTrue(trapArm.setArmGoalCommand(20, 10));
    m_driverController.button(12).onTrue(trapArm.setArmGoalCommand(0, -6));*/
   
  }


  
  private void initializeChooser(){
      chooser.addOption("Reta", new PathPlannerAuto("testeRetaPid"));
      chooser.addOption("AutonomoLadoEsquerdo", new PathPlannerAuto("autonomoLadoEsquerdo"));
      chooser.addOption("AutonomoLadoDireito", new PathPlannerAuto("autonomoLadoDireito"));
      chooser.addOption("AutonomoCentro", new PathPlannerAuto("autonomoCentro"));
      chooser.addOption("braco", new PathPlannerAuto("TesteBraco"));
      chooser.addOption("intake", new PathPlannerAuto("TesteIntake"));
      SmartDashboard.putData("CHOOSER", chooser);
  }   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }

  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
