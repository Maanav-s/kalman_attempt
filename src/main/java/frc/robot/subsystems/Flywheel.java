// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

  

  TalonFX main;

  private final LinearSystem<N1, N1, N1> m_flywheelPlant;
  private final KalmanFilter<N1, N1, N1> m_observer;
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller;
  private final LinearSystemLoop<N1, N1, N1> m_loop;

  /** Creates a new Flywheel. */
  public Flywheel() {
    main = new TalonFX(Constants.CAN_ID);
    main.configFactoryDefault();
    main.setSelectedSensorPosition(0);
    main.setInverted(TalonFXInvertType.CounterClockwise);
    main.setNeutralMode(NeutralMode.Coast);

    m_flywheelPlant = LinearSystemId.identifyVelocitySystem(Constants.kV, Constants.kA);
    m_observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        m_flywheelPlant,
        VecBuilder.fill(Constants.kModelAccuracy), // model
        VecBuilder.fill(Constants.kEncoderAccuracy), // encoder
        0.020); // periodic time?
    m_controller = new LinearQuadraticRegulator<>(
        m_flywheelPlant,
        VecBuilder.fill(Constants.kVelocityErrorTolerance), // qelms. Velocity error tolerance, in radians per second.
                                                            // Decrease
        // this to more heavily penalize state excursion, or make the controller behave
        // more
        // aggressively.
        VecBuilder.fill(Constants.kVoltageErrorTolerance), // relms. Control effort (voltage) tolerance. Decrease this
                                                           // to more
        // heavily penalize control effort, or make the controller less aggressive. 12
        // is a good
        // starting point because that is the (approximate) maximum voltage of a
        // battery.
        0.020); // Periodic time?
    m_loop = new LinearSystemLoop<>(
        m_flywheelPlant,
        m_controller,
        m_observer,
        12.0, // max voltage
        0.020); // periodic time?
  }

  @Override
  public void periodic() {

  }

  public void run() {
    main.set(ControlMode.PercentOutput, 0.3);
  }

  public void stop() {
    main.set(ControlMode.PercentOutput, 0);
  }
}
