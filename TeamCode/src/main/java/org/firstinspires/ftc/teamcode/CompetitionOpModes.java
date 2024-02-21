package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.autonomous.old.blue.BlueClose2Plus0;
import org.firstinspires.ftc.teamcode.autonomous.old.blue.BlueClose2Plus4AT;
import org.firstinspires.ftc.teamcode.autonomous.old.blue.BlueFar2Plus3Fast;
import org.firstinspires.ftc.teamcode.autonomous.old.blue.BlueFar2Plus3Slow;
import org.firstinspires.ftc.teamcode.autonomous.old.red.RedClose2Plus0;
import org.firstinspires.ftc.teamcode.autonomous.old.red.RedClose2Plus4;
import org.firstinspires.ftc.teamcode.autonomous.old.red.RedFar2Plus3;
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
        manager.register(metaForClass(BlueClose2Plus0.class, OpModeMeta.Flavor.AUTONOMOUS), BlueClose2Plus0.class);
        manager.register(metaForClass(BlueClose2Plus4AT.class, OpModeMeta.Flavor.AUTONOMOUS), BlueClose2Plus4AT.class);
        manager.register(metaForClass(BlueFar2Plus3Fast.class, OpModeMeta.Flavor.AUTONOMOUS), BlueFar2Plus3Fast.class);
        manager.register(metaForClass(BlueFar2Plus3Slow.class, OpModeMeta.Flavor.AUTONOMOUS), BlueFar2Plus3Slow.class);
        manager.register(metaForClass(RedClose2Plus0.class, OpModeMeta.Flavor.AUTONOMOUS), RedClose2Plus0.class);
        manager.register(metaForClass(RedClose2Plus4.class, OpModeMeta.Flavor.AUTONOMOUS), RedClose2Plus4.class);
        manager.register(metaForClass(RedFar2Plus3.class, OpModeMeta.Flavor.AUTONOMOUS), RedFar2Plus3.class);
    }
}
