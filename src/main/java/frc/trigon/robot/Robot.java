// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.phoenix6.Phoenix6Inputs;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    // --- ESP32 CAN test sender ---
    // Creates a "virtual CAN device" address in the WPILib CAN protocol.
    // deviceId must be 0-63. Pick an unused number for your LED controller.
    private final CAN esp32Can = new CAN(42, CAN.kTeamManufacturer, CAN.kTeamDeviceType);

    // 10-bit API ID (0-1023). This is like a "message type" for your packets.
    private static final int ESP32_API_ID = 0x123;

    // Payload to send (up to 8 bytes). We'll send a 1-byte "hello" + RGB test.
    private static final byte[] ESP32_TEST_PAYLOAD =
            new byte[]{ (byte) 0x42, (byte) 255, (byte) 0, (byte) 0 };


    public static final boolean IS_REAL = Robot.isReal();
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    Robot() {
        RobotConstants.init();
        configLogger();
        robotContainer = new RobotContainer();

        // Start repeating the test packet so the ESP32 can detect it.
        // This sends every 100ms automatically.
        esp32Can.writePacketRepeating(ESP32_TEST_PAYLOAD, ESP32_API_ID, 100);
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        Phoenix6Inputs.refreshAllInputs();
        commandScheduler.run();
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            commandScheduler.schedule(autonomousCommand);
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void testInit() {
        commandScheduler.cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        SimulationFieldHandler.update();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    private void configLogger() {
        if (RobotHardwareStats.isReplay()) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter(RobotConstants.LOGGING_PATH));
        }

        Logger.start();
    }
}
