package frc.robot.faults;

import java.util.LinkedList;

public class FaultManager {
    private class FaultStatus {
        public IFault Fault;
        public int Counts;
        public boolean Latched;

        public FaultStatus(IFault fault) {
            this.Fault = fault;
            Counts = 0;
            Latched = false;
        }
    }
    private LinkedList<FaultStatus> faults;

    public void addFault(IFault fault){
        faults.add(new FaultStatus(fault));
    }

    public void update() {
        for (FaultStatus status : faults) {
            status.Fault.CheckFault();

            if(status.Latched == false && status.Fault.IsFailed) {
                
            }
        }
    }
}
