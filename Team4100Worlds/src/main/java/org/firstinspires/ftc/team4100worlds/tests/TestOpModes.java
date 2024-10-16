package org.firstinspires.ftc.team4100worlds.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;

import java.util.Arrays;

public final class TestOpModes {
    public static final String GROUP = "testing";
    public static final boolean DISABLED = ScrappyConstants.IS_COMPETITION;

    private TestOpModes() {
    }

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

        manager.register(metaForClass(AngleErrorTest.class), AngleErrorTest.class);
        manager.register(metaForClass(BlueLeftInnerAuto.class), BlueLeftInnerAuto.class);
        manager.register(metaForClass(CircleTest.class), CircleTest.class);
        manager.register(metaForClass(ConceptAprilTagEasy.class), ConceptAprilTagEasy.class);
        manager.register(metaForClass(DistanceTest.class), DistanceTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);
        manager.register(metaForClass(PathChainTest.class), PathChainTest.class);
        manager.register(metaForClass(SampleRevBlinkinLedDriver.class), SampleRevBlinkinLedDriver.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                AngleErrorTest.class,
                BlueLeftInnerAuto.class,
                CircleTest.class,
                ConceptAprilTagEasy.class,
                DistanceTest.class,
                LocalizationTest.class,
                PathChainTest.class,
                SampleRevBlinkinLedDriver.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
