package com.example.MeepMeepTesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                .setConstraints(23.961260360229645, 30.0, Math.toRadians(97.8548008152838), Math.toRadians(60), 15.12)
                .followTrajectorySequence(drive ->
                        );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();


    }
}