"""用 Windows 本地 TTS 生成快速验证用 KWS 数据集。"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

from kws_common import (
    CACHE_DIR,
    RAW_DIR,
    RANDOM_SEED,
    SAMPLE_COUNT,
    SAMPLE_RATE,
    add_noise,
    configure_local_cache,
    ensure_output_dirs,
    fit_to_one_second,
    load_and_prepare_audio,
    load_labels,
    normalize_peak,
    write_json,
    write_wav_mono,
)


PHRASES = {
    "wake": ["小车小车", "小车 小车"],
    "start": ["开始", "启动", "开始执行"],
    "stop": ["停止", "停车", "停下"],
    "forward": ["前进", "向前", "往前走"],
    "backward": ["后退", "倒车", "往后退"],
    "move_left": ["左移", "向左平移", "往左移动"],
    "move_right": ["右移", "向右平移", "往右移动"],
    "turn_left": ["左转", "向左转", "往左"],
    "turn_right": ["右转", "向右转", "往右"],
    "select_cube": ["选择方块", "抓方块", "方块"],
    "select_cylinder": ["选择圆柱", "抓圆柱", "圆柱"],
    "select_sphere": ["选择球体", "抓球体", "球体"],
    "select_hippo": ["选择河马", "抓河马", "河马"],
    "select_cat": ["选择小猫", "抓小猫", "小猫"],
    "select_stone_pillar": ["选择石墩子", "抓石墩子", "石墩子"],
    "select_capybara": ["选择水豚", "抓水豚", "水豚"],
    "select_corn": ["选择玉米", "抓玉米", "玉米"],
    "select_jiaotailang": ["选择蕉太狼", "抓蕉太狼", "蕉太狼"],
    "select_dog": ["选择小狗", "抓小狗", "小狗"],
    "select_tnt": ["选择TNT", "抓TNT", "抓 TNT"],
    "unknown": ["你好", "天气", "测试", "结束", "打开", "关闭", "现在", "速度", "屏幕", "颜色"],
}


def get_chinese_voice_tokens():
    configure_local_cache()
    import win32com.client

    voice = win32com.client.Dispatch("SAPI.SpVoice")
    tokens = []
    voices = voice.GetVoices()
    for index in range(voices.Count):
        token = voices.Item(index)
        lang = str(token.GetAttribute("Language") or "")
        description = token.GetDescription()
        if lang.lower() == "804" or "chinese" in description.lower() or "zh" in token.Id.lower():
            tokens.append(token)
    if not tokens:
        for index in range(voices.Count):
            tokens.append(voices.Item(index))
    return tokens


def synthesize_to_wav(text: str, token, rate: int, volume: int, output_path: Path) -> None:
    import win32com.client

    def speak_once(force_16k: bool) -> None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if output_path.exists():
            output_path.unlink()

        voice = win32com.client.Dispatch("SAPI.SpVoice")
        stream = win32com.client.Dispatch("SAPI.SpFileStream")
        if force_16k:
            audio_format = win32com.client.Dispatch("SAPI.SpAudioFormat")
            # 22 = SAFT16kHz16BitMono，若语音不支持该格式则回退到默认格式后重采样。
            audio_format.Type = 22
            stream.Format = audio_format

        stream.Open(str(output_path), 3, False)
        try:
            voice.Voice = token
            voice.Rate = rate
            voice.Volume = volume
            voice.AudioOutputStream = stream
            voice.Speak(text, 0)
        finally:
            stream.Close()

    try:
        speak_once(True)
    except Exception:
        speak_once(False)


def generate_speech_sample(label: str, index: int, token, rng: random.Random, np_rng: np.random.Generator, tmp_dir: Path) -> Path:
    phrase = rng.choice(PHRASES[label])
    rate = rng.choice([-2, -1, 0, 1, 2])
    volume = rng.choice([70, 82, 95, 100])
    noise_level = rng.choice([0.0, 0.0015, 0.003, 0.005])
    target_peak = rng.choice([0.55, 0.65, 0.75, 0.85])

    tmp_wav = tmp_dir / f"{label}_{index:03d}_tts.wav"
    synthesize_to_wav(phrase, token, rate, volume, tmp_wav)
    audio = load_and_prepare_audio(tmp_wav)
    audio = fit_to_one_second(audio, rng)
    audio = normalize_peak(audio, target_peak)
    audio = add_noise(audio, np_rng, noise_level)

    output_path = RAW_DIR / label / f"{label}_{index:03d}.wav"
    write_wav_mono(output_path, audio, SAMPLE_RATE)
    return output_path


def generate_silence_sample(index: int, rng: random.Random, np_rng: np.random.Generator) -> Path:
    mode = rng.choice(["zero", "noise", "hum"])
    audio = np.zeros(SAMPLE_COUNT, dtype=np.float32)
    if mode == "noise":
        audio = np_rng.normal(0.0, rng.choice([0.001, 0.002, 0.004]), SAMPLE_COUNT).astype(np.float32)
    elif mode == "hum":
        t = np.arange(SAMPLE_COUNT, dtype=np.float32) / SAMPLE_RATE
        audio = (0.002 * np.sin(2.0 * np.pi * 50.0 * t)).astype(np.float32)
        audio = add_noise(audio, np_rng, 0.001)

    output_path = RAW_DIR / "silence" / f"silence_{index:03d}.wav"
    write_wav_mono(output_path, audio, SAMPLE_RATE)
    return output_path


def reset_dataset() -> None:
    if RAW_DIR.exists():
        shutil.rmtree(RAW_DIR)


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 TTS KWS 快速验证数据集")
    parser.add_argument("--samples-per-label", type=int, default=20, help="每个标签生成样本数")
    parser.add_argument("--seed", type=int, default=RANDOM_SEED, help="随机种子")
    parser.add_argument("--keep-existing", action="store_true", help="保留已有 raw 数据")
    args = parser.parse_args()

    ensure_output_dirs()
    if not args.keep_existing:
        reset_dataset()
    RAW_DIR.mkdir(parents=True, exist_ok=True)

    labels = load_labels()
    rng = random.Random(args.seed)
    np_rng = np.random.default_rng(args.seed)

    try:
        tokens = get_chinese_voice_tokens()
    except Exception as exc:
        print(f"获取 Windows TTS 语音失败: {exc}")
        return 1

    if not tokens:
        print("没有可用 TTS 语音。")
        return 1

    token_names = [token.GetDescription() for token in tokens]
    print("使用 TTS 语音:")
    for name in token_names:
        print(f"  - {name}")

    tmp_dir = CACHE_DIR / "tts_tmp"
    if tmp_dir.exists():
        shutil.rmtree(tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)

    generated = {}
    try:
        for label in labels:
            (RAW_DIR / label).mkdir(parents=True, exist_ok=True)
            generated[label] = 0
            for index in range(args.samples_per_label):
                if label == "silence":
                    generate_silence_sample(index, rng, np_rng)
                else:
                    speech_label = label if label in PHRASES else "unknown"
                    token = tokens[index % len(tokens)]
                    generate_speech_sample(speech_label, index, token, rng, np_rng, tmp_dir)
                    if speech_label != label:
                        src = RAW_DIR / speech_label / f"{speech_label}_{index:03d}.wav"
                        dst = RAW_DIR / label / f"{label}_{index:03d}.wav"
                        dst.parent.mkdir(parents=True, exist_ok=True)
                        src.replace(dst)
                generated[label] += 1
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    write_json(
        RAW_DIR / "dataset_manifest.json",
        {
            "sample_rate": SAMPLE_RATE,
            "duration_seconds": 1.0,
            "samples_per_label": args.samples_per_label,
            "labels": labels,
            "voices": token_names,
            "seed": args.seed,
            "generated": generated,
            "note": "TTS 数据集只用于快速验证链路，不代表真实麦克风效果。",
        },
    )

    print("生成完成:")
    for label in labels:
        count = len(list((RAW_DIR / label).glob("*.wav")))
        print(f"  {label}: {count} wav")
    return 0


if __name__ == "__main__":
    sys.exit(main())
