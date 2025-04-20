// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// hi

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private String autoName, newAutoName;

  private Field2d m_field = new Field2d();

  Optional<Alliance> ally = DriverStation.getAlliance();
  Optional<Alliance> newAlly;

  public void robotInit() {
    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // gets layout from elastic
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    updateSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    displayAuto();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    displayAuto();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    displayCurrentPath();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  // function to setup elastic
  private void updateSmartDashboard() {
    //get swerve from subsystem and calc velocity 
    var swerve = m_robotContainer.drivebase.getSwerveDrive();
    var vel = Math.hypot(swerve.getRobotVelocity().vxMetersPerSecond, swerve.getRobotVelocity().vyMetersPerSecond); 

    SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Robot Velocity", vel);

    // Update Field2d pose
    var robotPose = swerve.getPose();
    m_field.setRobotPose(robotPose);
  }

  private void displayAuto() {
    ally = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName | ally != newAlly) {
      newAlly = ally;
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        System.out.println("Displaying " + autoName);
        try {
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
            if (ally.isPresent()) {
              if (ally.get() == Alliance.Red) {
                poses.addAll(path.getAllPathPoints().stream()
                .map(point -> new Pose2d(Constants.Pose.feildFlip - point.position.getX(),Constants.Pose.feildFlipy - point.position.getY(), new Rotation2d()))
                .collect(Collectors.toList()));
                //Elastic.selectTab("RED");
              }
              if (ally.get() == Alliance.Blue) {
                poses.addAll(path.getAllPathPoints().stream()
                .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                .collect(Collectors.toList()));
                //Elastic.selectTab("BLUE");
              }
            }
            else {
              System.out.println("No alliance found");
              poses.addAll(path.getAllPathPoints().stream()
              .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
              .collect(Collectors.toList()));
            }
          }  
          m_field.getObject("path").setPoses(poses);
        } catch (IOException e) {
          e.printStackTrace();
        } catch (Exception e) {
          if (e instanceof ParseException) {
            e.printStackTrace();
          } else {
            e.printStackTrace();
          }
        }
      }
    }
  }

  private void displayCurrentPath() {
    List<Pose2d> poses = new ArrayList<>();
    PathPlannerPath path = m_robotContainer.drivebase.getLastPath();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        poses.addAll(path.getAllPathPoints().stream()
        .map(point -> new Pose2d(Constants.Pose.feildFlip - point.position.getX(),Constants.Pose.feildFlipy - point.position.getY(), new Rotation2d()))
        .collect(Collectors.toList()));
      }
      if (ally.get() == Alliance.Blue) {
        poses.addAll(path.getAllPathPoints().stream()
        .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
        .collect(Collectors.toList()));
      }
    }
    else {
      poses.addAll(path.getAllPathPoints().stream()
      .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
      .collect(Collectors.toList()));
    }
    m_field.getObject("path").setPoses(poses);
  }
}
