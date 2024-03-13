package org.firstinspires.ftc.team4100worlds.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;

public final class TestOpModes {
    public static final String GROUP = "testing";
    public static final boolean DISABLED = ScrappyConstants.IS_COMPETITION;

    private TestOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;
        manager.register(metaForClass(CircleTest.class), CircleTest.class);
        manager.register(metaForClass(PathChainTest.class), PathChainTest.class);
        manager.register(metaForClass(BlueLeftInnerAuto.class), BlueLeftInnerAuto.class);
    }
}
