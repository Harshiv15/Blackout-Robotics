package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 14)
                .setDimensions(15, 15)
                .setColorScheme(new ColorScheme() {
                    @Override
                    public boolean isDark() {
                        return true;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return Color.BLACK;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return Color.ORANGE;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return Color.ORANGE;
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return Color.RED;
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJCETORY_PATH_COLOR() {
                        return Color.ORANGE;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return Color.GRAY;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return Color.RED;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return Color.BLACK;
                    }

                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return Color.BLACK;
                    }
                })
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 70, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(30, 27, Math.toRadians(215)), Math.toRadians(215))
                                .back(5)
                                .lineTo(new Vector2d(35, 22))
                                .splineToSplineHeading(new Pose2d(57.5, 11.5, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.75f)
                .addEntity(myBot)
                .start();
    }
}