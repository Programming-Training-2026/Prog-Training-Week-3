// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SuperStructureCommands;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0); 
  private final KickerSubsystem kicker = new KickerSubsystem(); 
  private final ShooterSubsystem shooter = new ShooterSubsystem(); 
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem(); 
  private final IntakeSubsystem intake = new IntakeSubsystem(); 
  private final ExtensionSubsystem extension = new ExtensionSubsystem(); 
  private final TurretSubsystem turret = new TurretSubsystem(); 
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.rightTrigger().whileTrue(SuperStructureCommands.runShooting(kicker, shooter, spindexer)); 

    controller.a().onTrue(intake.runIntaking()); 
    controller.b().onTrue(intake.stopIntaking()); 

    controller.x().onTrue(extension.extend()); 
    controller.y().onTrue(extension.retract());
    
    controller.povLeft().whileTrue(turret.moveLeft()); 
    controller.povRight().whileTrue(turret.moveRight()); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
