// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CAN_ID = 46;

    public static final double kV = 0.12723;
    public static final double kA = 0.016509;

    public static final double kModelAccuracy = 0.0001;
    public static final double kEncoderAccuracy = 0.0001;

    public static final double kVelocityErrorTolerance = 1;
    public static final double kVoltageErrorTolerance = 1;
}
