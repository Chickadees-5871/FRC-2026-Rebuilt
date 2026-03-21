package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private static final double CLIMB_REST_POS = 150.0;
    private static final double CLIMB_ENGAGED_POS = 0.0;

    private SparkMax climbMotor;
    private PIDController climbController;

    public ClimbSubsystem(){
        climbMotor = new SparkMax(16, MotorType.kBrushless);
        climbController = new PIDController(0.15, 0, 0);
    }

    public Command pullUpCommand(){
        return new InstantCommand(() -> pullUp());
    }

    public Command releaseCommand(){
        return new InstantCommand(() -> release());
    }

    public void pullUp(){
        climbController.setSetpoint(CLIMB_ENGAGED_POS);
    }

    public void release(){
        climbController.setSetpoint(CLIMB_REST_POS);
    }

    @Override
    public void periodic(){
        double climbPower = clamp(climbController.calculate(climbMotor.getEncoder().getPosition()), -0.2, 0.2);
        climbMotor.set(climbPower);
    }

    public double clamp(double d, double min, double max){
        if(d > max) return max;
        if(d < min) return min;
        return d;
    }
}
