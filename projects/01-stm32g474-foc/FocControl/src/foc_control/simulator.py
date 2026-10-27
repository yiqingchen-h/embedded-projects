"""明确标记的离线演示板卡，仅供界面预览和测试，绝不访问 COM。"""

from __future__ import annotations

import math
import time

from .models import (
    ALGORITHMS,
    ENCODER_ALGORITHMS,
    FIRMWARE,
    FOC_ALGORITHMS,
    PROTOCOL,
    ParameterValue,
    specs_for,
)


class DemoTransport:
    """以相同 ASCII 协议提供确定性的电机演示，不代表实板验证。"""

    def __init__(
        self,
        firmware: str = FIRMWARE,
        algorithm: str = "SENSORLESS_FOC",
        g2208: bool = False,
        current_support: bool = False,
        position_support: bool = False,
        haptic_support: bool = False,
    ) -> None:
        """@brief 初始化独立算法块。@param firmware 模拟身份。@param algorithm 初始算法。
        @param g2208 使用当前七极对模型，仍为离线数据。@param current_support 模拟独立Iq契约。
        """
        self.firmware = firmware
        self.algorithm = algorithm
        self.g2208 = g2208
        self.current_support = current_support
        self.position_support = position_support
        self.haptic_support = haptic_support
        self.haptic_kind = "DETENT"
        self.haptic_detents = 24
        self.haptic_strength = 100
        self.haptic_damping = 2000
        self.haptic_session = 0
        self.haptic_updated = 0.0
        self.haptic_parameters = {
            "CURRENT_LIMIT": ParameterValue(500, 3, "A"),
            "RUN_TIME": ParameterValue(60000, 0, "MS"),
        }
        self.position_target = 0
        self.position_angle = 0.0
        self.position_zero_valid = False
        self.position_epoch = 0
        self.position_tol = 100
        self.position_updated = time.monotonic()
        self.position_parameters = {
            "POSITION_KP": ParameterValue(8000, 3, "1/S"),
            "SPEED_LIMIT": ParameterValue(60, 0, "RPM"),
            "RAMP": ParameterValue(300000, 3, "RPM/S"),
            "SPEED_KP": ParameterValue(1333, 3, "MA/RPM"),
            "SPEED_KI": ParameterValue(266667, 3, "MA/R/S"),
        }
        self.iq_target = 0
        self.iq_reference = 0.0
        self.iq_updated = time.monotonic()
        self.calibration = "VALID"
        self.calibrating = False
        self.source = "LOCAL"
        self.mode = "SPEED"
        self.run = "STOPPED"
        self.print_mode = "OFF"
        self.result = "OK"
        self.fault = "NONE"
        self.brightness = 50
        self.animation = 1
        self.closed = False
        self.save_delay = 0.02
        self.last_set = -100.0
        self.started = 0.0
        self.last_emit = 0.0
        self.incoming = bytearray()
        self.commands: list[str] = []
        self.blocks: dict[str, dict[str, ParameterValue]] = {}
        for algorithm in ALGORITHMS:
            profile = specs_for(algorithm)
            values = {key: definition.minimum for key, definition in profile.items()}
            values.update(POLE_PAIRS=3, SHUNT=50, RATED_VOLTAGE=12000, RATED_SPEED=5400)
            if algorithm in FOC_ALGORITHMS:
                values.update(
                    TARGET=2000,
                    CURRENT_LIMIT=1000,
                    RUN_TIME=20000,
                    SPEED_KP=400,
                    SPEED_KI=500,
                    RAMP=200000,
                    D_KP=1206,
                    Q_KP=1206,
                    D_KI=3970973,
                    Q_KI=3970973,
                )
            else:
                values.update(
                    TARGET=1000,
                    CURRENT_LIMIT=1500,
                    RUN_TIME=20000,
                    DUTY=200,
                    START_HZ=50000,
                    ALIGN_TIME=200,
                    RAMP=20000,
                    SPEED_LIMIT=5400,
                    ENCODER_DIR=1,
                    SPEED_KP=0,
                    SPEED_KI=0,
                )
            if g2208:
                values.update(
                    POLE_PAIRS=7,
                    TARGET=600,
                    CURRENT_LIMIT=500,
                    SPEED_LIMIT=1800,
                    RATED_SPEED=1800,
                    RATED_CURRENT=800,
                    RUN_TIME=5000,
                    DUTY=600,
                )
                if algorithm in FOC_ALGORITHMS:
                    values.update(RESISTANCE=10100, LD=1500, LQ=1500, SPEED_KP=100, SPEED_KI=200)
            if algorithm in ENCODER_ALGORITHMS:
                values.update(ENCODER_DIR=1, ELEC_ZERO=504)
            self.blocks[algorithm] = {
                key: ParameterValue(values[key], definition.decimals, definition.unit)
                for key, definition in profile.items()
            }

    @property
    def parameters(self) -> dict[str, ParameterValue]:
        """@brief 返回当前算法块。@return 参数。"""
        values = self.blocks[self.algorithm]
        if self.mode == "HAPTIC" and self.algorithm == "ENCODER_FOC":
            return {
                **{
                    key: value
                    for key, value in values.items()
                    if key in specs_for(self.algorithm, mode="HAPTIC")
                },
                **self.haptic_parameters,
            }
        if self.mode == "CURRENT" and self.algorithm == "ENCODER_FOC":
            return {
                **{key: values[key] for key in specs_for(self.algorithm, mode="CURRENT")},
                "TARGET": ParameterValue(self.iq_target, 3, "A"),
                "CURRENT_LIMIT": ParameterValue(500, 3, "A"),
            }
        if self.mode == "POSITION" and self.algorithm == "ENCODER_FOC":
            profile = specs_for(self.algorithm, mode="POSITION")
            return {
                **{key: value for key, value in values.items() if key in profile},
                **self.position_parameters,
                "TARGET": ParameterValue(self.position_target, 3, "DEG"),
                "CURRENT_LIMIT": values["CURRENT_LIMIT"],
            }
        return values

    def position_motion(self) -> tuple[float, int]:
        """只供演示的连续定位曲线，不代表真实控制算法或精度。"""
        now = time.monotonic()
        elapsed = max(0.0, now - self.position_updated)
        error = self.position_target / 1000 - self.position_angle
        rpm = max(-60.0, min(60.0, error * 8 / 6)) if self.run == "RUNNING" else 0.0
        self.position_angle += max(-abs(error), min(abs(error), rpm * 6 * elapsed))
        self.position_updated = now
        return rpm, round(2097152 * self.position_angle / 360)

    def current_motion(self) -> tuple[float, int]:
        """@brief 产生明确为演示的Iq斜坡及机械转速。@return 毫安参考和RPM。"""
        now = time.monotonic()
        change = max(
            -2000 * (now - self.iq_updated),
            min(2000 * (now - self.iq_updated), self.iq_target - self.iq_reference),
        )
        self.iq_reference += change
        self.iq_updated = now
        return self.iq_reference, round(self.iq_reference * 4)

    def emit(self, line: str) -> None:
        """@brief 排入一条完整响应。@param line 正文。"""
        self.incoming.extend((line + "\r\n").encode("ascii"))

    def stage(self) -> int:
        """@brief 取得演示启动阶段。@return 0..6。"""
        if self.run != "RUNNING":
            return 0
        if self.algorithm in ENCODER_ALGORITHMS:
            return 1 if self.calibrating else 6
        elapsed = time.monotonic() - self.started
        return (
            2
            if elapsed < 1.2
            else 3 if elapsed < 2.2 else 4 if elapsed < 2.5 else 5 if elapsed < 2.7 else 6
        )

    def read(self) -> bytes:
        """@brief 产生有界演示遥测并读取分包。@return 字节。"""
        if self.closed:
            raise OSError("演示连接已关闭")
        now = time.monotonic()
        if self.run == "RUNNING":
            if self.calibrating and now - self.started >= 0.05:
                self.calibrating, self.calibration, self.run = False, "VALID", "STOPPED"
            elif now - self.started >= self.parameters["RUN_TIME"].value / 1000:
                self.run = "STOPPED"
            elif (
                self.algorithm in FOC_ALGORITHMS
                and self.print_mode == "TRACE"
                and now - self.last_emit >= 0.02
            ):
                self.last_emit = now
                self.emit(self.trace())
        data = bytes(self.incoming[:4096])
        del self.incoming[:4096]
        return data

    def trace(self) -> str:
        """@brief 生成协议字段齐全的演示帧。@return TRACE 行。"""
        elapsed = time.monotonic() - self.started
        stage = self.stage()
        target = self.parameters.get("TARGET", ParameterValue(0, 0, "RPM")).value
        rpm = target * min(1, elapsed / 2.8) + (5 * math.sin(elapsed * 4) if stage == 6 else 0)
        poles = self.parameters["POLE_PAIRS"].value
        theta = elapsed * rpm * math.pi * poles / 30
        ia, ib = int(130 * math.sin(theta)), int(130 * math.sin(theta - 2 * math.pi / 3))
        iq = int(125 + 6 * math.sin(elapsed * 6))
        reference = 125
        if self.mode == "CURRENT":
            ref, rpm = self.current_motion()
            reference, iq = round(ref), round(ref + 2 * math.sin(elapsed * 6))
            theta = elapsed * rpm * math.pi * poles / 30
            ia, ib = round(iq * math.sin(theta)), round(iq * math.sin(theta - 2 * math.pi / 3))
        if self.mode == "POSITION":
            rpm, _ = self.position_motion()
            reference = iq = round(max(-500.0, min(500.0, rpm * 2)))
        sample, ticks = int(elapsed * 10000), int(elapsed * 170000000) & 0xFFFFFFFF
        values = [
            sample,
            ticks,
            sample + 1,
            stage,
            2048,
            2048,
            2048,
            ia,
            ib,
            -ia - ib,
            -ia - ib + 3,
            int(4 * math.sin(elapsed * 5)),
            iq,
            0,
            reference,
            int((theta % (2 * math.pi)) * 1000),
            int(rpm * math.pi * poles / 30 * 1000),
            10,
            4000,
            100,
            3000,
            1,
            11980,
            0,
            4150,
            0,
            3300,
        ]
        return "FOC_TRACE:" + ",".join(str(value) for value in values)

    def write(self, data: bytes) -> None:
        """@brief 响应相同命令并保留可审计发送列表。@param data 命令。"""
        command = data.decode("ascii").rstrip("\r\n")
        self.commands.append(command)
        words = command.split()
        now = time.monotonic()
        # 模拟板卡时限独立于主机读调用，查询时不能返回已过期的 RUNNING。
        if self.run == "RUNNING" and now - self.started >= self.parameters["RUN_TIME"].value / 1000:
            self.run = "STOPPED"
        if command == "VERSION":
            self.emit(f"OK VERSION FW={self.firmware} PROTOCOL={PROTOCOL}")
        elif command == "PING":
            self.emit("OK PING")
        elif command == "STATUS":
            if self.run == "RUNNING" and self.algorithm == "SENSORLESS_6STEP":
                self.emit("ERR RUNNING STOP TO READ")
            else:
                self.emit(
                    f"OK STATUS MOTOR={self.run} ALGORITHM={self.algorithm} MODE={self.mode} READY=1 OLED=DISABLED"
                )
        elif words[0] == "GET":
            key = words[1]
            if key == "HAPTIC_CAPS":
                self.emit(
                    "OK GET HAPTIC_CAPS=HAP1 TYPES=DETENT,DAMPED CLOCK=170000000"
                    if self.haptic_support
                    else "ERR GET ARGUMENT"
                )
                return
            if key == "POSITION_CAPS":
                self.emit(
                    "OK GET POSITION_CAPS=POS1 UNIT=DEG SCALE=1000 MULTITURN=1 HOLD=1 CLOCK=170000000"
                    if self.position_support
                    else "ERR GET ARGUMENT"
                )
                return
            if key in self.parameters:
                parameter = self.parameters[key]
                metadata = ""
                if self.current_support:
                    if key == "RUN_TIME":
                        metadata = " MIN=1000 MAX=2147483647 EDITABLE=1"
                    elif self.mode in ("CURRENT", "POSITION", "HAPTIC") and key in (
                        "TARGET",
                        "CURRENT_LIMIT",
                        "SPEED_LIMIT",
                        "SPEED_KP",
                        "SPEED_KI",
                        "RAMP",
                        "POSITION_KP",
                    ):
                        definition = specs_for(self.algorithm, mode=self.mode)[key]
                        metadata = f" MIN={definition.minimum} MAX={definition.maximum} EDITABLE={int(definition.editable)}"
                self.emit(
                    f"OK GET {key}={parameter.value} DECIMALS={parameter.decimals} UNIT={parameter.unit}{metadata}"
                )
            else:
                values = {
                    "ALGORITHM": self.algorithm,
                    "MODE": self.mode,
                    "SOURCE": self.source,
                    "FOC_PRINT": self.print_mode,
                    "RESULT": self.result,
                    "SAVE": "PENDING" if now - self.last_set < self.save_delay else "SAVED",
                    "BRIGHTNESS": str(self.brightness),
                    "ANIMATION": str(self.animation),
                }
                self.emit(f"OK GET {key}={values[key]}" if key in values else "ERR GET ARGUMENT")
        elif words[0] == "SET":
            key, value = words[1:3]
            if key == "ALGORITHM":
                self.algorithm = value
                if value != "ENCODER_FOC":
                    self.mode = "SPEED"
                self.emit(f"OK SET {key}={value}")
            elif key in ("MODE", "FOC_PRINT", "BRIGHTNESS", "ANIMATION"):
                if key == "MODE" and (
                    self.run == "RUNNING"
                    or value not in ("SPEED", "CURRENT", "POSITION", "HAPTIC")
                    or value == "HAPTIC"
                    and (not self.haptic_support or self.algorithm != "ENCODER_FOC")
                    or value == "POSITION"
                    and (not self.position_support or self.algorithm != "ENCODER_FOC")
                    or value == "CURRENT"
                    and (not self.current_support or self.algorithm != "ENCODER_FOC")
                ):
                    self.emit("ERR SET BUSY" if self.run == "RUNNING" else "ERR SET NOT_READY")
                    return
                attribute = {"FOC_PRINT": "print_mode"}.get(key, key.lower())
                setattr(
                    self, attribute, int(value) if key in ("BRIGHTNESS", "ANIMATION") else value
                )
                self.emit(f"OK SET {key}={value}")
            else:
                self.result = "OK"
                if key == "SOURCE":
                    self.source = value
                elif key in self.parameters:
                    if self.mode == "CURRENT" and (
                        key == "CURRENT_LIMIT" or key == "TARGET" and not -500 <= int(value) <= 500
                    ):
                        self.emit("ERR SET RANGE")
                        return
                    if (
                        self.run == "RUNNING"
                        and self.algorithm == "SENSORLESS_FOC"
                        and self.stage() != 6
                    ):
                        self.result = "FOC WAIT CLOSED"
                    else:
                        prior = self.parameters[key]
                        if self.mode == "HAPTIC" and key in self.haptic_parameters:
                            if self.run == "RUNNING":
                                self.emit("ERR SET BUSY")
                                return
                            self.haptic_parameters[key] = ParameterValue(
                                int(value), prior.decimals, prior.unit
                            )
                        elif self.mode == "POSITION" and key == "TARGET":
                            self.position_motion()
                            self.position_target = int(value)
                        elif self.mode == "POSITION" and key in self.position_parameters:
                            self.position_parameters[key] = ParameterValue(
                                int(value), prior.decimals, prior.unit
                            )
                        elif key == "TARGET" and self.mode == "CURRENT":
                            self.iq_target = int(value)
                        else:
                            self.blocks[self.algorithm][key] = ParameterValue(
                                int(value), prior.decimals, prior.unit
                            )
                else:
                    self.emit("ERR SET ARGUMENT")
                    return
                self.emit("OK SET PENDING")
            # 位置目标是易失指令，和真实固件一致，不触发Flash保存等待。
            if key != "FOC_PRINT" and not (self.mode == "POSITION" and key == "TARGET"):
                self.last_set = now
        elif command.startswith("MOTOR FOC HAPTIC") and self.haptic_support:
            if len(words) > 3:
                if self.mode != "HAPTIC" or len(words) != 8 or words[3] != "CONFIG":
                    self.emit("ERR FOC ARGUMENT")
                    return
                kind, count, strength, damping = (
                    words[4],
                    int(words[5]),
                    int(words[6]),
                    int(words[7]),
                )
                if (
                    kind not in ("DETENT", "DAMPED")
                    or not 2 <= count <= 256
                    or not 0 <= strength <= 800
                    or not 0 <= damping <= 10000
                ):
                    self.emit("ERR FOC RANGE")
                    return
                self.haptic_kind, self.haptic_detents, self.haptic_strength, self.haptic_damping = (
                    kind,
                    count,
                    strength,
                    damping,
                )
                self.haptic_updated = now
                self.last_set = now
            self.emit(
                f"OK FOC HAPTIC KIND={self.haptic_kind} DETENTS={self.haptic_detents} DETENT_MA={self.haptic_strength} DAMPING_UA_RPM={self.haptic_damping} LIMIT_MA={self.haptic_parameters['CURRENT_LIMIT'].value} RUN_MS={self.haptic_parameters['RUN_TIME'].value} VALID=1 RUN={int(self.run == 'RUNNING' and self.mode == 'HAPTIC')} TRANSITION={int(now - self.haptic_updated < .2)} SESSION={self.haptic_session} RAW=123456 TICKS={int(max(0, now - self.started) * 170000000) & 0xFFFFFFFF} POS_LO=0 POS_HI=0 CENTER_LO=0 CENTER_HI=0 INDEX_LO=0 INDEX_HI=0 REF_MA=0 IQ_MA=0 RPM_MILLI=0"
            )
        elif command == "MOTOR FOC POSITION" and self.position_support:
            rpm, counts = self.position_motion()
            error = self.position_target - round(self.position_angle * 1000)
            arrived = (
                self.run == "RUNNING"
                and self.position_zero_valid
                and abs(error) <= self.position_tol
            )
            ticks = round(now * 170000000) & 0xFFFFFFFF
            self.emit(
                f"OK FOC POSITION VALID={int(self.position_zero_valid)} ZERO_ID={self.position_epoch} RAW={counts & 2097151} POS_LO={counts & 0xFFFFFFFF} POS_HI={counts >> 32} TICKS={ticks} TARGET_MDEG={self.position_target} REF_MRPM={round(rpm * 1000)} ERROR_MDEG={error} ARRIVED={int(arrived)} TOL_MDEG={self.position_tol} RUN={int(self.run == 'RUNNING' and self.mode == 'POSITION')}"
            )
        elif command == "MOTOR FOC POSITION ZERO" and self.position_support:
            if self.run == "RUNNING" or self.mode != "POSITION" or self.calibration != "VALID":
                self.emit("ERR FOC BUSY")
            else:
                self.position_angle = 0.0
                self.position_target = 0
                self.position_zero_valid = True
                self.position_epoch += 1
                self.emit("OK FOC POSITION ZERO")
        elif command.startswith("MOTOR FOC POSITION TOLERANCE ") and self.position_support:
            tolerance_value = int(words[-1])
            if self.run == "RUNNING" or tolerance_value not in (100, 500):
                self.emit("ERR FOC RANGE")
            else:
                self.position_tol = tolerance_value
                self.last_set = now
                self.emit("OK FOC POSITION TOLERANCE")
        elif command == "START":
            if self.mode == "POSITION" and not self.position_zero_valid:
                self.result = "POSITION ZERO REQUIRED"
                self.emit("OK START PENDING")
                return
            if self.algorithm in ENCODER_ALGORITHMS and self.calibration != "VALID":
                self.result = "ENCODER CAL REQUIRED"
                self.emit("OK START PENDING")
                return
            self.run, self.started, self.result = "RUNNING", now, "OK"
            self.iq_reference, self.iq_updated = 0.0, now
            self.position_updated = now
            self.emit("OK START PENDING")
        elif command == "STOP":
            if self.calibrating:
                self.calibrating, self.calibration = False, "REQUIRED"
            self.run, self.result = "STOPPED", "OK"
            self.emit("OK STOP PENDING")
        elif command == "CLEAR":
            self.run, self.result, self.fault = "STOPPED", "OK", "NONE"
            self.emit("OK CLEAR PENDING")
        elif command == "MOTOR FOC DRIVE":
            stage_names = (
                "STOPPED",
                "CALIBRATE",
                "ALIGN",
                "I_F_RAMP",
                "QUALIFY",
                "BLEND",
                "CLOSED",
            )
            active = self.run == "RUNNING"
            iq, rpm = (125.0, self.parameters.get("TARGET", ParameterValue(0, 0, "RPM")).value)
            if self.mode == "HAPTIC":
                iq, rpm = 0.0, 0
            if self.mode == "CURRENT":
                iq, rpm = self.current_motion()
            if self.mode == "POSITION":
                rpm, _ = self.position_motion()
                rpm = round(rpm)
            metadata = (
                f" MODE={self.mode} TARGET_IQ_MA={self.iq_target} IQ_SLEW_MA_S=2000"
                if self.current_support
                else ""
            )
            self.emit(
                f"OK MOTOR FOC DRIVE STATE={stage_names[self.stage()]} ACTIVE={int(active)} "
                f"CLOSED_SEEN={int(self.stage() == 6)} AWAITING_COMMIT=0 EST_RPM={rpm} "
                f"ID_MA=0 IQ_MA={round(iq)} REF_D_MA=0 REF_Q_MA={round(iq)} RPM_MILLI={rpm * 1000 + 12} FAULT={self.fault}{metadata}"
            )
        elif command == "MOTOR ENCODER":
            if not self.g2208 and self.algorithm not in ENCODER_ALGORITHMS:
                self.emit("ERR MOTOR ARGUMENT")
            else:
                self.emit(
                    f"OK MOTOR ENCODER STATE={'CALIBRATING' if self.calibrating else 'RUNNING' if self.run == 'RUNNING' else 'STOPPED'} "
                    f"CALIBRATION={self.calibration} RPM={self.parameters.get('TARGET', ParameterValue(0, 0, 'RPM')).value} RPM_VALID=1 FAULT={self.fault}"
                )
        elif command == "MOTOR ENCODER CAL":
            self.position_zero_valid = False
            self.position_target = 0
            self.calibrating, self.calibration, self.run, self.started = (
                True,
                "REQUIRED",
                "RUNNING",
                now,
            )
            self.emit("OK ENCODER CAL RUNNING")
        elif command == "MOTOR FOC REST":
            self.emit(
                f"OK MOTOR FOC REST OUTPUT_DISABLED={int(self.run != 'RUNNING')} STILL=1 FAULT={self.fault}"
            )
        elif command == "STORAGE":
            self.emit("OK STORAGE SAVE=SAVED RESULT=OK MATCH=1")
        elif command == "MOTOR":
            self.emit(f"OK MOTOR BUS_MV=11980 FAULT={self.fault} SAVE=SAVED")
        elif command in ("MOTOR RUN", "MOTOR BEMF"):
            self.emit(f"OK {command} TICKS=200000 FAULT={self.fault}")
        else:
            self.emit("ERR " + words[0] + " ARGUMENT")

    def close(self) -> None:
        """@brief 关闭演示连接。"""
        self.closed = True
