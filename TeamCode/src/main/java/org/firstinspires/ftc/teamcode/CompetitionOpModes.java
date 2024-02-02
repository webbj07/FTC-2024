package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.autonomous.blue.BlueFar2PlusOne;
import org.firstinspires.ftc.teamcode.teleop.ScrappyTeleOp;

public final class CompetitionOpModes {
    public static final String GROUP = "competition";

    private CompetitionOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, OpModeMeta.Flavor flavor) {
        String name = cls.getSimpleName();

        try {
            name = String.valueOf(cls.getField("name"));
        } catch (NoSuchFieldException e) {}

        return new OpModeMeta.Builder()
                .setName(name)
                .setGroup(GROUP)
                .setFlavor(flavor)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        manager.register(metaForClass(ScrappyTeleOp.class, OpModeMeta.Flavor.TELEOP), ScrappyTeleOp.class);
        manager.register(metaForClass(BlueFar2PlusOne.class, OpModeMeta.Flavor.AUTONOMOUS), BlueFar2PlusOne.class);
    }
}
