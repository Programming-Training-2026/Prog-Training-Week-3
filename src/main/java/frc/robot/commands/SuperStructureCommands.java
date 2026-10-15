package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class SuperStructureCommands {
    public static Command runShooting(KickerSubsystem kicker, ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        // Shooter wind up, wait 0.75 seconds, and then run all three of the subsystems combined
        return Commands.parallel(
            shooter.runShooting(), 
            Commands.sequence(
                Commands.waitSeconds(0.75), 
                Commands.parallel(kicker.runKicker(), spindexer.spin()))); 
        // Commands.sequence(kicker.runKicker(), shooter.runShooting(), spindexer.spin()); runs each command in order
        // Commands.parallel(kicker.runKicker(), shooter.runShooting(), spindexer.spin()) runs each command at the same time and ends when
        //                                                                                they all finish                                                                               
        // Commands.deadline(kicker.runKicker(), shooter.runShooting(), spindexer.spin()) // runs all at the same time but ends when the 
        //                                                                                   first command stops                                                                  
    }
}
