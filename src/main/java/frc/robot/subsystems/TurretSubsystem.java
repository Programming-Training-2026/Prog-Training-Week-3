package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase{
    private final TalonFX turretMotor; 

    private static final int[] RED_HUB_IDS = {8, 9, 10, 11};
    private static final int[] BLUE_HUB_IDS = {24, 25, 26, 27};
    // Using a Hashset is better, but I decided to use arrays

    private final String LIMELIGHT_NAME = "limelight-turret"; 

    private final PIDController pidController = new PIDController(0, 0, 0); 
    private double kp = 0.03; 
    public TurretSubsystem() {
        turretMotor = new TalonFX(24); 
        configureMotor(); 
        pidController.setP(kp); 
        pidController.setTolerance(0.5);
        this.setDefaultCommand(centerAprilTag()); 
    }

    public void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration(); 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        turretMotor.getConfigurator().apply(config); 
    }

    public Command moveLeft() {
        return run(
            () -> turretMotor.setVoltage(3)
        ).finallyDo(
            (interrupted) -> turretMotor.stopMotor()
        ); 
    }

    public Command moveRight() {
        return run(
            () -> turretMotor.setVoltage(-3)
        ).finallyDo(
            (interrupted) -> turretMotor.stopMotor()
        ); 
    }

    // Implement a rudimentary turret auto align
    public Command centerAprilTag() {
        return run(
            () -> {
                if (LimelightHelpers.getTV(LIMELIGHT_NAME) 
                    && isId((int)LimelightHelpers.getFiducialID(LIMELIGHT_NAME))) {
                    
                    double offset = LimelightHelpers.getTX(LIMELIGHT_NAME); 
                    double output = pidController.calculate(offset, 0); 
                    turretMotor.setVoltage(output); 
                } else {
                    turretMotor.setVoltage(0); 
                }
            }
        )
        .beforeStarting(pidController::reset)
        .finallyDo(
            (interrupted) -> turretMotor.stopMotor()
        ); 
    }

    public boolean isId(int id) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) { 
            for (int i : BLUE_HUB_IDS) {
                if (i == id) {
                    return true;
                }
            }
        }
        
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            for (int i : RED_HUB_IDS) {
                if (i == id) {
                    return true;
                }
            }
        }
        return false; 
    }
}