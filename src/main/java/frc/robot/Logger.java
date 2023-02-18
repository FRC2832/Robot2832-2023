package frc.robot;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Logger {
    private static HashMap<String,Object> items = new HashMap<String,Object>();

    private NetworkTable currentTable;
    //private NetworkTable commandTable;
    private NetworkTable tempTable;
    private NetworkTable faultTable;
    private NetworkTable sensorTable;
    //pressure, solenoids, compressor, sensors, task rate, bus load

    //devices to add: pigeon, PDP, PH
    
    public Logger() {
        currentTable = NetworkTableInstance.getDefault().getTable("Motor_Currents");
        tempTable = NetworkTableInstance.getDefault().getTable("Motor_Temps");
        faultTable = NetworkTableInstance.getDefault().getTable("Device_Faults");
        sensorTable = NetworkTableInstance.getDefault().getTable("Sensors");
    }

    public static void RegisterTalon(String name, BaseTalon talon) {
        items.put(name, talon);
    }

    public static void RegisterSensor(String name, DoubleSupplier value) {
        items.put(name, value);
    }

    public void periodic() {
        // Print keys
        for (String i : items.keySet()) {
            var item = items.get(i);

            if(item instanceof BaseTalon) {
                readTalon(i, (BaseTalon)item);
            } else if(item instanceof DoubleSupplier) {
                readInput(i, (DoubleSupplier)item);
            } else {
                //unknown table
            }
        }
    }

    private void readTalon(String name, BaseTalon talon) {
        String faultStr;
        //reading the raw bits because we know there are faults not in Faults (aka Neutral Brake Current Limit)
        var handle = talon.getHandle();
        var faultBits = MotControllerJNI.GetFaults(handle);
        var error = talon.getLastError();
        var sfaultBits = MotControllerJNI.GetStickyFaults(handle);
        var error2 = talon.getLastError();

        if (error != ErrorCode.OK) {
            faultStr = error.name();
        } else if (error2 != ErrorCode.OK) {
            faultStr = error2.name();
        } else if (faultBits > 0) {
            faultStr = "Fault: " + readTalonFaults(faultBits);
        } else if (sfaultBits > 0) {
            faultStr = "Sticky Fault: " + readTalonFaults(sfaultBits);
        } else {
            faultStr = "Ok";
        }
        currentTable.getEntry(name).setDouble(talon.getStatorCurrent());
        faultTable.getEntry(name).setString(faultStr);
        tempTable.getEntry(name).setDouble(talon.getTemperature());
    }

    private void readInput(String name, DoubleSupplier func) {
        sensorTable.getEntry(name).setNumber(func.getAsDouble());
    }

    private String readTalonFaults(int bits) {
        Faults faults = new Faults();
        String retVal;
        faults.update(bits);

        StringBuilder work = new StringBuilder();
        work.append(faults.UnderVoltage ? "UnderVoltage " : "");
        work.append(faults.ForwardLimitSwitch ? "ForwardLimitSwitch " : "");
        work.append(faults.ReverseLimitSwitch ? "ReverseLimitSwitch " : "");
        work.append(faults.ForwardSoftLimit ? "ForwardSoftLimit " : "");
        work.append(faults.ReverseSoftLimit ? "ReverseSoftLimit " : "");
        work.append(faults.ResetDuringEn ? "ResetDuringEn " : "");
        work.append(faults.SensorOverflow ? "SensorOverflow " : "");
        work.append(faults.SensorOutOfPhase ? "SensorOutOfPhase " : "");
        work.append(faults.HardwareESDReset ? "HardwareESDReset " : "");
        work.append(faults.RemoteLossOfSignal ? "RemoteLossOfSignal " : "");
        work.append(faults.APIError ? "APIError " : "");
        work.append(faults.SupplyOverV ? "SupplyOverV " : "");
        work.append(faults.SupplyUnstable ? "SupplyUnstable " : "");
        retVal = work.toString();

        if(retVal.length() == 0) {
            retVal = "Unknown Fault " + bits;
        }
        return retVal;
    }
}
