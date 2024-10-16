package org.firstinspires.ftc.team4100worlds.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team4100worlds.ScrappyConstants;

import java.util.Arrays;

public final class TuningOpModes {
    public static final String GROUP = "tuning";
    public static final boolean DISABLED = ScrappyConstants.IS_COMPETITION;

    private TuningOpModes() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
            .setName(cls.getSimpleName())
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
            .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        manager.register(metaForClass(CurvedBackAndForth.class), CurvedBackAndForth.class);
        manager.register(metaForClass(ForwardVelocityTuner.class), ForwardVelocityTuner.class);
        manager.register(metaForClass(ForwardZeroPowerAccelerationTuner.class), ForwardZeroPowerAccelerationTuner.class);
        manager.register(metaForClass(HeadingPIDTuner.class), HeadingPIDTuner.class);
        manager.register(metaForClass(LateralZeroPowerAccelerationTuner.class), LateralZeroPowerAccelerationTuner.class);
        manager.register(metaForClass(ParallelSlidesVelTuner.class), ParallelSlidesVelTuner.class);
        manager.register(metaForClass(StrafeVelocityTuner.class), StrafeVelocityTuner.class);
        manager.register(metaForClass(StraightBackAndForth.class), StraightBackAndForth.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                CurvedBackAndForth.class,
                ForwardVelocityTuner.class,
                ForwardZeroPowerAccelerationTuner.class,
                HeadingPIDTuner.class,
                LateralZeroPowerAccelerationTuner.class,
                ParallelSlidesVelTuner.class,
                StrafeVelocityTuner.class,
                StraightBackAndForth.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
