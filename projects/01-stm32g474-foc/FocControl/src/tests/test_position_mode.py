"""位置模式的原始坐标、协议、独立参数、零点和真实界面离线回归。"""

from __future__ import annotations

import asyncio
import os
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from foc_control import controller as module
from foc_control.controller import Controller
from foc_control.models import BoardSnapshot, ControlError, ParameterValue, specs_for
from foc_control.position_feedback import PositionBuffer, PositionPoint, PositionSample
from foc_control.protocol import command_bytes
from foc_control.simulator import DemoTransport
from foc_control.storage import Preset
from foc_control.window import MainWindow
from PySide6.QtWidgets import QApplication, QFileDialog


def response(counts: int = 0, ticks: int = 0, epoch: int = 1) -> str:
    """构造完整POS1回执，负数按有符号高字传输。"""
    return (
        f"OK FOC POSITION VALID=1 ZERO_ID={epoch} RAW={counts & 2097151} "
        f"POS_LO={counts & 0xFFFFFFFF} POS_HI={counts >> 32} TICKS={ticks} "
        "TARGET_MDEG=720000 REF_MRPM=0 ERROR_MDEG=0 ARRIVED=1 TOL_MDEG=100 RUN=1"
    )


@pytest.mark.parametrize("counts", (-4194304, -524288, 0, 2097152, 4194304, 0x100000001))
def test_original_signed_multi_turn_counts(counts: int) -> None:
    """正负多圈原码保持，不限制到单圈或通过RPM积分重建。"""
    sample = PositionSample.parse(response(counts))
    assert sample.counts == counts
    assert sample.angle_deg == counts * 360 / 2097152


def test_source_ticks_wrap_and_gap_never_interpolates() -> None:
    """DWT回绕展开，零点变化和长缺口断线，停机不拼接休眠时间。"""
    buffer = PositionBuffer(3)
    first = PositionSample.parse(response(10, 0xFFFF0000))
    assert buffer.append(first) == PositionPoint(0, first, False)
    second = replace(first, ticks=(first.ticks + 8500000) & 0xFFFFFFFF, counts=20)
    point = buffer.append(second)
    assert point is not None and point.continuous and point.seconds == 0.05
    assert buffer.append(second) is None
    gap = buffer.append(replace(second, ticks=(second.ticks + 34000000) & 0xFFFFFFFF))
    assert gap is not None and not gap.continuous
    assert buffer.append(replace(second, running=False)) is None
    assert buffer.append(replace(second, zero_id=2, ticks=100)) is not None


def test_position_readonly_and_units() -> None:
    """只读模式只能查询，角度毫度与电流安培不能混用。"""
    assert command_bytes("MOTOR FOC POSITION", True)
    with pytest.raises(ControlError):
        command_bytes("MOTOR FOC POSITION ZERO", True)
    with pytest.raises(ControlError):
        command_bytes("MOTOR FOC POSITION TOLERANCE 500", True)
    profile = specs_for("ENCODER_FOC", mode="POSITION")
    assert profile["TARGET"].encode("-90.125") == ParameterValue(-90125, 3, "DEG")
    assert profile["TARGET"].encode("720").value == 720000
    assert profile["CURRENT_LIMIT"].encode("0.6") == ParameterValue(600, 3, "A")
    with pytest.raises(ControlError):
        profile["CURRENT_LIMIT"].encode("0.801")


def test_zero_target_modes_preset_and_active_stop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """零点先备份；三模式目标独立；位置在线改目标、预设和STOP均走正式控制器。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    async def scenario() -> None:
        port = DemoTransport("demo-position", "ENCODER_FOC", True, True, True)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            assert client.state.position_available
            await client.select_mode("POSITION")
            assert client.state.parameters["TARGET"] == ParameterValue(0, 3, "DEG")
            await client.apply({"CURRENT_LIMIT": ParameterValue(600, 3, "A")})
            assert client.state.parameters["CURRENT_LIMIT"].value == 600
            with pytest.raises(ControlError, match="机械零点"):
                await client.start_motor({})
            assert "START" not in port.commands
            await client.zero_position()
            assert client.state.position is not None and client.state.position.valid
            assert len(list((tmp_path / "restore_backups").glob("*.json"))) == 2
            await client.apply(
                {
                    "TARGET": ParameterValue(-90000, 3, "DEG"),
                    "SPEED_LIMIT": ParameterValue(30, 0, "RPM"),
                }
            )
            preset = await client.capture("position_negative")
            preset.save(tmp_path / "position.json")
            assert Preset.load(tmp_path / "position.json").mode == "POSITION"
            await client.start_motor({})
            await client.read_drive()
            await client.apply({"TARGET": ParameterValue(720000, 3, "DEG")})
            await client.read_position()
            assert port.run == "RUNNING" and port.position_target == 720000
            with pytest.raises(ControlError):
                await client.zero_position()
            await client.stop_motor()
            await client.position_tolerance(500)
            assert client.state.position is not None and client.state.position.tolerance_deg == 0.5
            await client.apply({"SPEED_LIMIT": ParameterValue(60, 0, "RPM")})
            await client.restore(preset)
            assert client.state.parameters["SPEED_LIMIT"].value == 30
            await client.select_mode("CURRENT")
            assert client.state.parameters["TARGET"] == ParameterValue(0, 3, "A")
            assert client.state.parameters["CURRENT_LIMIT"].value == 500
            assert not client.state.parameter_specs["CURRENT_LIMIT"].editable
            await client.apply({"TARGET": ParameterValue(-20, 3, "A")})
            await client.select_mode("SPEED")
            assert client.state.parameters["TARGET"] == ParameterValue(600, 0, "RPM")
            await client.select_mode("POSITION")
            assert client.state.parameters["TARGET"] == ParameterValue(-90000, 3, "DEG")
            assert client.state.parameters["CURRENT_LIMIT"].value == 600
            assert "CLEAR" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_old_position_firmware_keeps_current_limit_readonly(tmp_path: Path) -> None:
    """旧固件无电流范围元数据时保留监视/定位，不能向其发送扩流命令。"""

    class OldPositionPort(DemoTransport):
        """模拟1.9.1位置限流只读且不发布元数据的真实回执。"""

        def emit(self, line: str) -> None:
            """只删除旧版本确实不存在的电流能力尾部。"""
            if line.startswith("OK GET CURRENT_LIMIT="):
                line = line.split(" MIN=", 1)[0]
            super().emit(line)

    async def scenario() -> None:
        port = OldPositionPort("demo-old-position", "ENCODER_FOC", True, True, True)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.select_mode("POSITION")
            assert client.state.control_available
            assert not client.state.parameter_specs["CURRENT_LIMIT"].editable
            with pytest.raises(ControlError):
                await client.apply({"CURRENT_LIMIT": ParameterValue(600, 3, "A")})
            assert "SET CURRENT_LIMIT 600" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_position_plot_keeps_raw_points_on_pause_and_export(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """真实Qt控件暂停继续接收，CSV保留原码和误差，不导出滤波结果。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    try:
        sample = PositionSample.parse(response(-524288, 170000000))
        panel = window.position_panel
        panel.pause.setChecked(True)
        panel.receive([PositionPoint(0, sample, False)])
        panel.receive([PositionPoint(0.05, replace(sample, ticks=178500000, counts=-524280), True)])
        assert len(panel.points) == 2
        panel.pause.setChecked(False)
        path = tmp_path / "raw_position.csv"
        monkeypatch.setattr(
            QFileDialog, "getSaveFileName", lambda *_args: (str(path), "CSV (*.csv)")
        )
        panel.export_csv()
        text = path.read_text(encoding="utf-8-sig")
        assert "-524288" in text and "-90.0" in text
        assert panel.actual_curve.getData()[1][-1] == -524280 * 360 / 2097152
    finally:
        window.close()
        app.processEvents()


@pytest.mark.parametrize(
    "calibration,force,expected", (("REQUIRED", False, 1), ("VALID", False, 0), ("VALID", True, 1))
)
def test_one_click_initialization_keeps_drafts_and_uses_one_backup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, calibration: str, force: bool, expected: int
) -> None:
    """@brief 初次、复用及强制标定共用一次事务，保存先于置零且目标最后恢复。
    @param tmp_path 备份目录。@param monkeypatch 保存替身。@param calibration 初始资格。
    @param force 是否强制重标。@param expected 预期标定次数。
    """
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    class FlashGapPort(DemoTransport):
        """模拟持久参数写入导致机械坐标失效。"""

        def write(self, data: bytes) -> None:
            """@brief 保留真实应答并使配置写入撤销旧坐标。@param data 串口字节。"""
            super().write(data)
            if data.decode("ascii").startswith(("SET RUN_TIME ", "SET SPEED_LIMIT ")):
                self.position_zero_valid = False

    async def scenario() -> None:
        """@brief 使用正式Controller完成初始化，不调用实际START。"""
        port = FlashGapPort("demo-init", "ENCODER_FOC", True, True, True)
        port.mode, port.calibration, port.position_target = "POSITION", calibration, 720000
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.initialize_position(
                {
                    "TARGET": ParameterValue(-90125, 3, "DEG"),
                    "RUN_TIME": ParameterValue(8000, 0, "MS"),
                    "SPEED_LIMIT": ParameterValue(30, 0, "RPM"),
                },
                force_calibration=force,
            )
            assert client.state.mode == "POSITION" and client.state.encoder_calibration == "VALID"
            assert client.state.position is not None and client.state.position.valid
            assert port.position_target == -90125 and port.parameters["RUN_TIME"].value == 8000
            assert port.run == "STOPPED" and client.state.busy == ""
            assert port.commands.count("MOTOR ENCODER CAL") == expected
            zero = port.commands.index("MOTOR FOC POSITION ZERO")
            assert port.commands.index("SET RUN_TIME 8000") < zero
            assert port.commands.index("SET SPEED_LIMIT 30") < zero
            assert port.commands.index("SET TARGET -90125") > zero
            backups = list((tmp_path / "restore_backups").glob("*.json"))
            assert len(backups) == 1
            before = Preset.load(backups[0])
            assert before.mode == "POSITION" and before.parameters["TARGET"].value == 720000
            assert "START" not in port.commands and "CLEAR" not in port.commands
            await client.initialize_position()
            assert port.position_target == -90125
            assert port.commands.count("MOTOR ENCODER CAL") == expected
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "draft",
    (
        {"TARGET": ParameterValue(1, 0, "RPM")},
        {"CURRENT_LIMIT": ParameterValue(801, 3, "A")},
        {"MISSING": ParameterValue(1, 0, "")},
    ),
)
def test_invalid_initialization_draft_never_energizes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, draft: dict[str, ParameterValue]
) -> None:
    """@brief 无效整份草稿在标定或写入前拒绝。@param tmp_path 临时目录。
    @param monkeypatch 保存替身。@param draft 非法草稿。
    """
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    async def scenario() -> None:
        """@brief 检查只有读取发生，不能产生标定或置零副作用。"""
        port = DemoTransport("demo-init", "ENCODER_FOC", True, True, True)
        port.mode, port.calibration = "POSITION", "REQUIRED"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            offset = len(port.commands)
            with pytest.raises(ControlError):
                await client.initialize_position(draft)
            assert all(
                not command.startswith(
                    ("SET ", "START", "MOTOR ENCODER CAL", "MOTOR FOC POSITION ZERO")
                )
                for command in port.commands[offset:]
            )
            assert not (tmp_path / "restore_backups").exists()
        finally:
            await client.session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("failure", ("calibration", "configuration"))
def test_initialization_failure_never_continues_to_zero_or_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure: str
) -> None:
    """@brief 标定或配置写入失败保留备份，禁止继续置零或自动清故障。
    @param tmp_path 临时目录。@param monkeypatch 保存替身。@param failure 故障环节。
    """
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    class FailedInitPort(DemoTransport):
        """在指定初始化环节提供真实类型的失败回执。"""

        def write(self, data: bytes) -> None:
            """@brief 注入一次执行失败。@param data 请求字节。"""
            command = data.decode("ascii").strip()
            if failure == "configuration" and command == "SET SPEED_LIMIT 30":
                self.commands.append(command)
                self.emit("ERR SET NOT_READY")
                return
            super().write(data)
            if failure == "calibration" and command == "MOTOR ENCODER CAL":
                self.run, self.calibrating, self.fault = "STOPPED", False, "ENC CAL ZERO"

    async def scenario() -> None:
        """@brief 实际执行失败路径并核对后续命令被截断。"""
        port = FailedInitPort("demo-init", "ENCODER_FOC", True, True, True)
        port.mode, port.calibration = "POSITION", (
            "REQUIRED" if failure == "calibration" else "VALID"
        )
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            with pytest.raises(ControlError, match="ENC CAL ZERO|NOT_READY"):
                await client.initialize_position({"SPEED_LIMIT": ParameterValue(30, 0, "RPM")})
            assert len(list((tmp_path / "restore_backups").glob("*.json"))) == 1
            assert "MOTOR FOC POSITION ZERO" not in port.commands
            assert "START" not in port.commands and "CLEAR" not in port.commands
            assert port.run == "STOPPED" and client.state.busy == ""
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_stop_cancels_whole_initialization_before_zero(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 七点过程中STOP取消后续保存和置零。@param tmp_path 临时目录。@param monkeypatch 保存替身。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    async def scenario() -> None:
        """@brief 取消正在标定的初始化任务并确认输出关闭。"""
        port = DemoTransport("demo-init", "ENCODER_FOC", True, True, True)
        port.mode, port.calibration = "POSITION", "REQUIRED"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            task = asyncio.create_task(client.initialize_position())
            while "MOTOR ENCODER CAL" not in port.commands:
                if task.done():
                    await task
                    pytest.fail("初始化没有进入标定阶段")
                await asyncio.sleep(0.002)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            await client.stop_motor()
            assert client.state.output_disabled and port.calibration == "REQUIRED"
            assert "MOTOR FOC POSITION ZERO" not in port.commands
            assert "START" not in port.commands and "CLEAR" not in port.commands
            assert client.state.busy == ""
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_saved_and_volatile_position_target_do_not_sleep_again(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 已保存查询与停机位置目标无固定等待，真实配置保存仍受原等待约束。
    @param tmp_path 临时目录。@param monkeypatch 保存窗口替身。
    """

    async def scenario() -> None:
        """@brief 将保存窗口设为十秒，确认只读与易失目标可在短超时内完成。"""
        port = DemoTransport("demo-init", "ENCODER_FOC", True, True, True)
        port.mode = "POSITION"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            monkeypatch.setattr(module, "SAVE_WAIT", 10)
            await asyncio.wait_for(client.saved(), 0.5)
            await asyncio.wait_for(client.apply({"TARGET": ParameterValue(720000, 3, "DEG")}), 0.5)
            await asyncio.wait_for(client.saved(), 0.5)
            assert port.position_target == 720000 and client.state.save == "SAVED"
            offset = len(port.commands)
            with pytest.raises(TimeoutError):
                await asyncio.wait_for(
                    client.apply({"SPEED_LIMIT": ParameterValue(30, 0, "RPM")}), 0.1
                )
            assert "SET SPEED_LIMIT 30" in port.commands[offset:]
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_position_settings_and_single_initialization_button(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 容差位于设置页，带草稿可一次初始化，配置修改不会误走直接启动。
    @param tmp_path 临时目录。@param monkeypatch 用户投递替身。
    """
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    port = DemoTransport("demo-ui", "ENCODER_FOC", True, True, True)
    port.mode = "POSITION"
    state = BoardSnapshot(
        connected=True,
        firmware="1.9.3-g2208-pos",
        protocol="1.4",
        algorithm="ENCODER_FOC",
        mode="POSITION",
        source="REMOTE",
        run="STOPPED",
        control_available=True,
        ready=True,
        fault="NONE",
        output_disabled=True,
        still=True,
        encoder_available=True,
        encoder_calibration="REQUIRED",
        position_available=True,
        position=replace(PositionSample.parse(response()), valid=False),
        parameters=port.parameters,
        parameter_specs=specs_for("ENCODER_FOC", port.parameters, "POSITION"),
    )
    submitted: list[tuple[str, object]] = []
    monkeypatch.setattr(
        window, "submit", lambda action, payload=None: submitted.append((action, payload))
    )
    try:
        window.receive_state(state)
        window.show()
        app.processEvents()
        assert window.calibrate_button.isEnabled() and not window.start_button.isEnabled()
        assert window.calibrate_button.text() == "一键初始化（标定并置零）"
        assert not hasattr(window, "position_zero_button")
        settings = window.tabs.widget(window.tabs.count() - 1)
        assert settings.isAncestorOf(window.position_tolerance_combo)
        assert not window.main_splitter.widget(0).isAncestorOf(window.position_tolerance_combo)
        window.editors["TARGET"].edit.setText("-90")
        window.calibrate_button.click()
        assert submitted.pop() == (
            "position_initialize",
            {"TARGET": ParameterValue(-90000, 3, "DEG")},
        )
        window.editors["RUN_TIME"].edit.setText("8")
        assert window.apply_button.text() == "应用参数并置零"
        assert not window.start_button.isEnabled()
        window.apply_button.click()
        assert submitted.pop()[0] == "position_initialize"
        window.discard_drafts()
        window.tabs.setCurrentWidget(settings)
        window.position_tolerance_combo.setCurrentIndex(1)
        assert submitted.pop() == ("position_tolerance", 500)
        window.operation_failed("position_tolerance", "模拟容差写入失败")
        assert window.position_tolerance_combo.currentData() == 100
        assert window.banner.text() == "模拟容差写入失败"
        assert window.banner.isVisible()
        window.recalibrate_button.click()
        assert submitted.pop() == ("recalibrate", {})
        window.receive_state(replace(state, read_only=True))
        assert (
            not window.calibrate_button.isEnabled()
            and not window.position_tolerance_combo.isEnabled()
        )
        assert "只读" in window.control_reason.text()
        assert not submitted
    finally:
        window.close()
        app.processEvents()


def test_encoder_calibration_applies_draft_before_energizing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """@brief 六步占空比等标定参数先保存，再执行通电标定。
    @param tmp_path 临时目录。@param monkeypatch 保存窗口替身。
    """
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    async def scenario() -> None:
        """@brief 检查参数应用和标定顺序，并确认只有一次备份。"""
        port = DemoTransport("demo-cal-draft", "ENCODER_6STEP", True, True, True)
        port.calibration = "REQUIRED"
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.calibrate_encoder({"DUTY": ParameterValue(500, 1, "%")})
            assert port.commands.index("SET DUTY 500") < port.commands.index("MOTOR ENCODER CAL")
            assert client.state.encoder_calibration == "VALID" and port.run == "STOPPED"
            assert len(list((tmp_path / "restore_backups").glob("*.json"))) == 1
            assert "START" not in port.commands
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_zero_saves_configuration_before_origin_and_keeps_volatile_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """模拟参数Flash停采；新零点必须晚于配置保存，目标写入不得破坏零点。"""
    monkeypatch.setattr(module, "SAVE_WAIT", 0.03)

    class FlashGapTransport(DemoTransport):
        """只模拟真实配置写Flash造成的原码间断。"""

        def write(self, data: bytes) -> None:
            super().write(data)
            if data.decode("ascii").startswith(("SET RUN_TIME ", "SET SPEED_LIMIT ")):
                self.position_zero_valid = False

    async def scenario() -> None:
        port = FlashGapTransport("demo-position", "ENCODER_FOC", True, True, True)
        client = Controller(port, tmp_path)
        try:
            await client.initialize()
            await client.select_mode("POSITION")
            await client.zero_position(
                {
                    "RUN_TIME": ParameterValue(8000, 0, "MS"),
                    "SPEED_LIMIT": ParameterValue(30, 0, "RPM"),
                    "TARGET": ParameterValue(-90000, 3, "DEG"),
                }
            )
            assert client.state.position is not None and client.state.position.valid
            assert port.position_target == -90000
            zero = port.commands.index("MOTOR FOC POSITION ZERO")
            assert port.commands.index("SET RUN_TIME 8000") < zero
            assert port.commands.index("SET SPEED_LIMIT 30") < zero
            assert port.commands.index("SET TARGET -90000") > zero
            backups = list((tmp_path / "restore_backups").glob("before_position_zero_*.json"))
            assert len(backups) == 1
            assert Preset.load(backups[0]).parameters["RUN_TIME"].value == 5000
            await client.start_motor({})
            assert port.run == "RUNNING"
            await client.stop_motor()
        finally:
            await client.session.close()

    asyncio.run(scenario())


def test_real_position_window_builds_all_parameter_groups(tmp_path: Path) -> None:
    """真实位置快照必须构建位置环分组及全部编辑项，不能仅后台流程通过。"""
    app = QApplication.instance() or QApplication([])
    window = MainWindow(tmp_path)
    port = DemoTransport("demo-position", "ENCODER_FOC", True, True, True)
    port.mode = "POSITION"
    try:
        state = BoardSnapshot(
            connected=True,
            firmware="1.9.1-g2208-pos",
            protocol="1.4",
            algorithm="ENCODER_FOC",
            mode="POSITION",
            source="REMOTE",
            run="STOPPED",
            control_available=True,
            ready=True,
            fault="NONE",
            output_disabled=True,
            still=True,
            encoder_available=True,
            encoder_calibration="VALID",
            position_available=True,
            position=PositionSample.parse(response()),
            parameters=port.parameters,
            parameter_specs=specs_for("ENCODER_FOC", port.parameters, "POSITION"),
        )
        window.receive_state(state)
        assert set(window.editors) == set(port.parameters)
        assert window.editors["POSITION_KP"].definition.group == "位置环"
        assert window.calibrate_button.isEnabled()
        assert window.start_button.isEnabled()
    finally:
        window.close()
        app.processEvents()
