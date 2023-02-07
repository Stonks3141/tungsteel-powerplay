package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.util.LiftPosition

@Config
class Lift(val hwMap: HardwareMap) : SubsystemBase() {
    var emergencyStop = false
    var level = Level.Floor
        set(value) {
            level = value
            setHeight(level.pos)
        }
    var offset = LiftPosition(0)

    private val motor = MotorGroup(
            MotorEx(hwMap, "lift0", Motor.GoBILDA.RPM_435),
            MotorEx(hwMap, "lift1", Motor.GoBILDA.RPM_435),
    )
    init {
        motor.setInverted(true)
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        motor.setRunMode(Motor.RunMode.VelocityControl)
        motor.setVeloCoefficients(kP, kI, kD)
        motor.resetEncoder()
    }

    private val pidf = PIDFController(kP, kI, kD, kF)
    init {
        pidf.setTolerance(tE, tD)
        pidf.setSetPoint(0)
    }

    private val dashboard = FtcDashboard.getInstance()
    private val telemetry = dashboard.getTelemetry()

    /**
     * Should be set as the default command.
     */
    fun update() {
        sendTelemetry()
        val avgPos = LiftPosition(LiftPosition.ticksToMm(motor.getPositions().filterNotNull().average()))
        val speedTarget = pidf.calculate(avgPos.ticks(), level.pos.ticks())
        if (emergencyStop || pidf.atSetPoint()) {
            motor.stopMotor()
        } else {
            motor.set(speedTarget)
        }
        telemetry.addData("PIDF calc target", speedTarget)
    }

    /**
     * Moves the lift up 1 level (saturating).
     */
    fun up() {
        level = level.up()
    }

    /**
     * Moves the lift down 1 level (saturating).
     */
    fun down() {
        level = level.down()
    }

    /**
     * An "emergency stop" if tuning makes the lift go crazy
     * Toggle forces the lift to 0 power
     */
    fun toggleStop() {
        emergencyStop = !emergencyStop
    }

    /**
     * Set the height of the lift.
     * @param pos the position to set
     */
    fun setHeight(pos: LiftPosition) {
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setTolerance(tE, tD);
        val maxHeight = LiftPosition(960)
        // Range.clip protects from inadvertently setting it too high or too low
        pidf.setSetPoint(Range.clip(pos.ticks() + offset.ticks(), 0, maxHeight.ticks()));
    }

    fun targetHeight() = offset.mm() + level.pos.mm()

    /**
     * Sends telemetry data for tuning/debugging purposes. Can be graphed with FTC Dashboard
     * which is pretty nifty
     * The FTC Dashboard address is http://192.168.43.1:8080/dash
     */
    fun sendTelemetry() {
        telemetry.addData("Lift Position (mm)", positionAvg().mm())
        telemetry.addData("Lift Target (mm)", level.pos.mm())
        telemetry.addData("Lift Error(mm)", positionAvg().mm() - level.pos.mm())
        telemetry.addData("Lift Velocity", motor.getVelocity()) // Ticks/second?
        telemetry.update()
    }

    enum class Level(val pos: LiftPosition) {
        // TODO: tune these to actual positions, in MILLIMETERS
        Floor(LiftPosition(0.0)),
        Short(LiftPosition(250.0)),
        Medium(LiftPosition(500.0)),
        Long(LiftPosition(960.0));

        fun up() = when (this) {
            Floor -> Short
            Short -> Medium
            Medium -> Long
            Long -> Long
        }

        fun down() = when (this) {
            Floor -> Floor
            Short -> Floor
            Medium -> Short
            Long -> Medium
        }
    }

    companion object {
        var kP = 0.04
        var kI = 0.0001
        var kD = 0.0008
        var kF = 0.0001
        var tE = 5.0
        var tD = 5.0
    }
}
