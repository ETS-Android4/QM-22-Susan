package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@AnalogSensorType
@DeviceProperties(name = "Sharp Distance", xmlTag = "SharpDistanceSensor", description = "sharp")
public class SharpDistance implements DistanceSensor, AnalogSensor {
    private final AnalogInputController analogInputController;
    private final int physicalPort;

    public SharpDistance(AnalogInputController analogInputController, int physicalPort) {
        this.analogInputController = analogInputController;
        this.physicalPort = physicalPort;
    }


    public double getMaxVoltage() {
        // The sensor itself is a 5v sensor, reporting analog values from 1.7v to 5v. However, depending
        // on the level conversion hardware that might be between us and the sensor, that may get shifted
        // to a different range. We'll assume that we only ever shift *down* in range, not up, so we
        // can take the min of the sensor's natural level and what the input controller can do.
        final double sensorMaxVoltage = 5.0;
        return Math.min(sensorMaxVoltage, analogInputController.getMaxAnalogInputVoltage());
    }


    @Override
    public double readRawVoltage() {
        return analogInputController.getAnalogInputVoltage(physicalPort);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return 10;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Sharp Distance";
    }

    @Override
    public String getConnectionInfo() {
        return analogInputController.getConnectionInfo() + "; analog port " + physicalPort;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
