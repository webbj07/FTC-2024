package org.firstinspires.ftc.team4100worlds;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.team4100worlds.autonomous.blue.close.BlueClose2Plus2TrussSafe;
import org.firstinspires.ftc.team4100worlds.autonomous.blue.close.BlueClose2Plus4Truss;
import org.firstinspires.ftc.team4100worlds.autonomous.blue.close.BlueClose2Plus5;
import org.firstinspires.ftc.team4100worlds.autonomous.blue.far.BlueFar2Plus3Safe;
import org.firstinspires.ftc.team4100worlds.autonomous.red.close.RedClose2Plus5;
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
        manager.register(metaForClass(BlueClose2Plus2TrussSafe.class, OpModeMeta.Flavor.AUTONOMOUS), BlueClose2Plus2TrussSafe.class);
        manager.register(metaForClass(BlueClose2Plus4Truss.class, OpModeMeta.Flavor.AUTONOMOUS), BlueClose2Plus4Truss.class);
        manager.register(metaForClass(BlueClose2Plus5.class, OpModeMeta.Flavor.AUTONOMOUS), BlueClose2Plus5.class);
        manager.register(metaForClass(BlueFar2Plus3Safe.class, OpModeMeta.Flavor.AUTONOMOUS), BlueFar2Plus3Safe.class);
        manager.register(metaForClass(RedClose2Plus5.class, OpModeMeta.Flavor.AUTONOMOUS), RedClose2Plus5.class);

        // TeleOp
        manager.register(metaForClass(ScrappyTeleOp.class, OpModeMeta.Flavor.TELEOP), ScrappyTeleOp.class);
    }
}
