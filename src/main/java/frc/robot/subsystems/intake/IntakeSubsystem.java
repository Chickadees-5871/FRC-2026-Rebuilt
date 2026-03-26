package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motorMain = new SparkMax(15 ,MotorType.kBrushless);
    private final SparkMax motorExtendOne = new SparkMax(16, MotorType.kBrushless);
    private final SparkMax motorExtendTwo = new SparkMax(17, MotorType.kBrushless);

    private double power = 0.41;
    private int reverse = 1;
    private boolean isDown = false;

    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
    
        config.follow(16, true);

        motorExtendTwo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorExtendOne.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorMain.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startCommand() {
        return new RunCommand(
            () -> motorMain.set(power * reverse), 
            this
        );
    }

    public Command stopCommand() {
        return new InstantCommand(() -> motorMain.set(0), this);
    }

    public Command toggleReverseCommand() {
        return new RunCommand(() -> {
            reverse *= -1;
            motorMain.set(power * reverse);
        }).finallyDo(() -> {
            reverse *= -1;
            motorMain.set(power * reverse);
        });
    }

    public Command extendCommand() {
        return new RunCommand(
            () -> {
                motorExtendOne.set(-0.25);
                System.out.println("Extending");
            }, this).withTimeout(2.0) 
            .finallyDo(() -> {
                isDown = true;
                motorExtendOne.set(0);
            });
    }

    public Command retractCommand() {
        return new RunCommand(() -> motorExtendOne.set(0.8), this)
            .finallyDo(() -> motorExtendOne.set(0));
    }

    public void periodic(){
        if(isDown){
            Commands.runOnce(() -> startCommand());
            isDown = false;
        }
    }
}
