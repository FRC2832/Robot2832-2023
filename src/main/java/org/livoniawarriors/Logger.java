package org.livoniawarriors;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class Logger implements Runnable {
    private final double VOLTS_PER_PSI = 1.931/100; //2.431V at 100psi

    private Notifier notify;
    private static HashMap<String,Object> items = new HashMap<String,Object>();
    private static PowerDistribution pdp;
    private static String[] pdpNames;
    private static PneumaticHub ph;

    private NetworkTable currentTable;
    private NetworkTable tempTable;
    private NetworkTable faultTable;
    private NetworkTable stickyTable;
    private NetworkTable sensorTable;
    private static NetworkTable taskTimings;
    //solenoids, compressor

    //devices to add: pigeon, cancoder
    
    public Logger() {
        notify = new Notifier(this);
        notify.startPeriodic(0.1);
        notify.setName("Logging");

        currentTable = NetworkTableInstance.getDefault().getTable("Motor_Currents");
        tempTable = NetworkTableInstance.getDefault().getTable("Motor_Temps");
        faultTable = NetworkTableInstance.getDefault().getTable("Device_Faults");
        stickyTable = NetworkTableInstance.getDefault().getTable("Device_Sticky_Faults");
        sensorTable = NetworkTableInstance.getDefault().getTable("Sensors");
        taskTimings = NetworkTableInstance.getDefault().getTable("Task_Timings_ms");
    }

    public static void RegisterTalon(String name, BaseTalon talon) {
        items.put(name, talon);
    }

    public static void RegisterSensor(String name, DoubleSupplier value) {
        items.put(name, value);
    }

    public static void RegisterPdp(PowerDistribution pdp, String[] channelNames) {
        Logger.pdp = pdp;
        pdpNames = channelNames;
    }

    public static void RegisterPneumaticHub(PneumaticHub ph) {
        Logger.ph = ph;
    }

    public static void RegisterLoopTimes(Robot robot) {
        new LoopTimeLogger(robot, taskTimings);
    }

    public void run() {
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

        if(pdp != null) {
            for(int i=0; i<pdpNames.length; i++) {
                if(pdpNames[i] != null) {
                    currentTable.getEntry("PDP Current " + pdpNames[i]).setDouble(pdp.getCurrent(i));
                }
            }
            sensorTable.getEntry("PDP Voltage").setDouble(pdp.getVoltage());
            sensorTable.getEntry("PDP Total Current").setDouble(pdp.getTotalCurrent());
            tempTable.getEntry("PDP").setDouble(pdp.getTemperature());
            faultTable.getEntry("PDP").setString(readFaultStruct(pdp.getFaults()));
            stickyTable.getEntry("PDP").setString(readFaultStruct(pdp.getStickyFaults()));
        }

        if(ph != null) {
            var volts = ph.getAnalogVoltage(0);
            sensorTable.getEntry("Pressure Sensor").setDouble((volts - 0.5) / VOLTS_PER_PSI);
            sensorTable.getEntry("Pressure Sensor Voltage").setDouble(volts);
            currentTable.getEntry("Compressor").setDouble(ph.getCompressorCurrent());
            faultTable.getEntry("PH").setString(readFaultStruct(ph.getFaults()));
            stickyTable.getEntry("PH").setString(readFaultStruct(ph.getStickyFaults()));
        }

        var canStatus = RobotController.getCANStatus();
        taskTimings.getEntry("CAN Bandwidth").setDouble(canStatus.percentBusUtilization);
        taskTimings.getEntry("CAN Bus Off Count").setDouble(canStatus.busOffCount);
        taskTimings.getEntry("CAN RX Error Count").setDouble(canStatus.receiveErrorCount);
        taskTimings.getEntry("CAN Tx Error Count").setDouble(canStatus.transmitErrorCount);
        taskTimings.getEntry("CAN Tx Full Count").setDouble(canStatus.txFullCount);

        sensorTable.getEntry("Rio 3.3V Voltage").setDouble(RobotController.getVoltage3V3());
        sensorTable.getEntry("Rio 5V Voltage").setDouble(RobotController.getVoltage5V());
        sensorTable.getEntry("Rio 6V Voltage").setDouble(RobotController.getVoltage6V());
        sensorTable.getEntry("Rio 3.3V Current").setDouble(RobotController.getCurrent3V3());
        sensorTable.getEntry("Rio 5V Current").setDouble(RobotController.getCurrent5V());
        sensorTable.getEntry("Rio 6V Current").setDouble(RobotController.getCurrent6V());
    }

    private void readTalon(String name, BaseTalon talon) {
        String faultStr;
        String sFaultStr;
        //reading the raw bits because we know there are faults not in Faults (aka Neutral Brake Current Limit)
        var handle = talon.getHandle();
        var faultBits = MotControllerJNI.GetFaults(handle);
        var error = talon.getLastError();
        var sfaultBits = MotControllerJNI.GetStickyFaults(handle);
        var error2 = talon.getLastError();

        if (error != ErrorCode.OK) {
            faultStr = error.name();
        } else if (faultBits > 0) {
            faultStr = readTalonFaults(faultBits);
        } else {
            faultStr = "Ok";
        }

        if (error2 != ErrorCode.OK) {
            sFaultStr = error2.name();
        } else if (sfaultBits > 0) {
            sFaultStr = readTalonFaults(sfaultBits);
        } else {
            sFaultStr = "Ok";
        }

        currentTable.getEntry(name).setDouble(talon.getStatorCurrent());
        faultTable.getEntry(name).setString(faultStr);
        stickyTable.getEntry(name).setString(sFaultStr);
        tempTable.getEntry(name).setDouble(talon.getTemperature());
    }

    private void readInput(String name, DoubleSupplier func) {
        sensorTable.getEntry(name).setNumber(func.getAsDouble());
    }

    private String readTalonFaults(int bits) {
        Faults faults = new Faults();
        String retVal;

        faults.update(bits);
        retVal = readFaultStruct(faults);
        if(retVal.length() == 0) {
            retVal = "Unknown Fault " + bits;
        }

        return retVal;
    }

    private String readFaultStruct(Object obj) {
        StringBuilder work = new StringBuilder();

        //iterate through all the fields and find the boolean ones
        Field fieldlist[] = obj.getClass().getDeclaredFields();
        for (int i = 0; fieldlist.length > i; i++) {
            Field fld = fieldlist[i];
            if (fld.getType().equals(boolean.class)) {
                try {
                    boolean value = fld.getBoolean(obj);
                    if(value) {
                        work.append(fld.getName() + " ");
                    }
                } catch (IllegalArgumentException e) {
                    System.out.println("IllegalFaultReadArg");
                } catch (IllegalAccessException e) {
                    System.out.println("IllegalFaultReadAccess");
                }
            }
        }

        //check if string is empty
        if(work.length() == 0) {
            work.append("Ok");
        }
        return work.toString();
    }
}
