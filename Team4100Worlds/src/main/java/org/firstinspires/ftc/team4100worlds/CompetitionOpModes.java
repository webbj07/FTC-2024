package org.firstinspires.ftc.team4100worlds;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team4100worlds.teleop.ScrappyTeleOp;

public final class CompetitionOpModes {
    public static final String GROUP = "competition";

    private CompetitionOpModes() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, OpModeMeta.Flavor flavor) {
        String name = cls.getSimpleName();

        try {
            name = String.valueOf(cls.getField("name"));
        } catch (NoSuchFieldException e) {
        }

        return new OpModeMeta.Builder()
            .setName(name)
            .setGroup(GROUP)
            .setFlavor(flavor)
            .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        // Autonomous

        // TeleOp
        manager.register(metaForClass(ScrappyTeleOp.class, OpModeMeta.Flavor.TELEOP), ScrappyTeleOp.class);
    }
}
