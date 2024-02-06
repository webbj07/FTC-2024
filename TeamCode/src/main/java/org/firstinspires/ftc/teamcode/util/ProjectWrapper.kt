package org.firstinspires.ftc.teamcode.util

import kotlin.math.sqrt
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.project

fun calculateProjectionDistance(path: PosePath, actualPose: Vector2d, dispGuess: Double): Double {
    return project(path, actualPose, dispGuess);
}

fun calculateDisplacement(lastPose: Pose2d, currentPose: Pose2d): Double {
    val xDifference = currentPose.position.x - lastPose.position.x
    val yDifference = currentPose.position.y - lastPose.position.y

    return sqrt(xDifference * xDifference + yDifference * yDifference)
}