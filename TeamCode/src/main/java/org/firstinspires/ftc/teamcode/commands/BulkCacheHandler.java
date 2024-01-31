package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkCacheHandler extends CommandBase {
    private final List<LynxModule> m_hubs;

    public BulkCacheHandler(HardwareMap hwMap) {
        m_hubs = hwMap.getAll(LynxModule.class);
    }

    @Override
    public void initialize() {
        m_hubs.forEach((hub) -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    @Override
    public void execute() {
        m_hubs.forEach(LynxModule::clearBulkCache);
    }
}
