from __future__ import annotations

import asyncio
import argparse
import random
import shutil
import tempfile
import wave
import subprocess
from pathlib import Path



# ============================================================
# 1. 基本配置
# ============================================================

AI_DIR = Path(__file__).resolve().parents[1]

# 生成到 dataset_real/auto；结果经人工筛选后加入 raw 训练数据
OUTPUT_ROOT = AI_DIR / "dataset_real" / "auto"

# 生成数量
NUM_PER_LABEL = 100
NUM_UNKNOWN = 150
NUM_SILENCE = 150

# 并发数量。Edge-TTS 是在线服务，不建议太高
CONCURRENCY = 4

# 输出音频格式
TARGET_SAMPLE_RATE = 16000

# 是否清空旧的 auto 数据
CLEAR_OLD_OUTPUT = False

# 是否使用当前 edge-tts 可用的全部中文音色
USE_ALL_CHINESE_VOICES = True

# 使用哪些中文地区音色
# zh-CN：大陆普通话
# zh-HK：港普/粤语风格
# zh-TW：台湾普通话
VOICE_LOCALE_PREFIXES = [
    "zh-CN",
    "zh-TW",
]


# ============================================================
# 2. 命令词配置
# ============================================================

COMMAND_TEXTS = {
    "wake": [
        "你好小车",
        "你好，小车",
        "小车你好",
        "小车小车",
    ],

    "start": [
        "开始",
        "开始任务",
        "开始执行",
        "启动",
    ],

    "stop": [
        "停止",
        "停下",
        "停止任务",
        "别动",
    ],

    "forward": [
        "前进",
        "向前",
        "往前走",
        "向前移动",
    ],

    "backward": [
        "后退",
        "向后",
        "往后退",
        "向后移动",
    ],

    "move_left": [
        "左平移",
        "向左平移",
        "左移",
        "往左移",
    ],

    "move_right": [
        "右平移",
        "向右平移",
        "右移",
        "往右移",
    ],

    "turn_left": [
        "左转",
        "向左转",
        "往左转",
    ],

    "turn_right": [
        "右转",
        "向右转",
        "往右转",
    ],

    "select_cube": [
        "抓取方块",
        "取方块",
        "抓方块",
        "拿方块",
    ],

    "select_cylinder": [
        "抓取圆柱",
        "抓圆柱",
        "取圆柱",
        "拿圆柱",
    ],

    "select_sphere": [
        "抓取圆球",
        "抓圆球",
        "取圆球",
        "拿圆球",
    ],

    "select_hippo": [
        "抓取河马",
        "抓河马",
        "取河马",
        "拿河马",
    ],

    "select_cat": [
        "抓取小猫",
        "抓小猫",
        "取小猫",
        "拿小猫",
    ],

    "select_stone_pillar": [
        "抓取石墩子",
        "抓石墩子",
        "取石墩子",
        "拿石墩子",
    ],

    "select_capybara": [
        "抓取水豚",
        "抓水豚",
        "取水豚",
        "拿水豚",
    ],

    "select_corn": [
        "抓取玉米",
        "抓玉米",
        "取玉米",
        "拿玉米",
    ],

    "select_jiaotailang": [
        "抓取蕉太狼",
        "抓蕉太狼",
        "取蕉太狼",
        "拿蕉太狼",
    ],

    "select_dog": [
        "抓取小狗",
        "抓小狗",
        "取小狗",
        "拿小狗",
    ],

    "select_tnt": [
        "抓取炸弹",
        "抓炸弹",
        "取炸弹",
        "拿炸弹",
    ],
}


UNKNOWN_TEXTS = [
    "今天天气不错",
    "这个东西放在哪里",
    "我正在测试麦克风",
    "请帮我打开电脑",
    "这个声音有点小",
    "你能听到我说话吗",
    "我们等一下再开始",
    "这个程序运行得很快",
    "前面的桌子上有东西",
    "我想看一下结果",
    "这个不是控制命令",
    "随便说一句话",
    "测试一下普通语音",
    "今天下午要调试代码",
    "这个数据还需要重新采集",
    "你看一下这个结果",
    "这里可能有一点问题",
    "我们先暂停一下",
    "这个不是小车指令",
    "我现在只是随便说话",
]


# 如果 USE_ALL_CHINESE_VOICES = False，则只使用这里的音色
PREFERRED_VOICES = [
    "zh-CN-XiaoxiaoNeural",
    "zh-CN-XiaoyiNeural",
    "zh-CN-YunjianNeural",
    "zh-CN-YunxiNeural",
    "zh-CN-YunyangNeural",
    "zh-CN-liaoning-XiaobeiNeural",
    "zh-CN-shaanxi-XiaoniNeural",
    "zh-HK-HiuMaanNeural",
    "zh-HK-WanLungNeural",
    "zh-TW-HsiaoChenNeural",
    "zh-TW-YunJheNeural",
]


# ============================================================
# 3. WAV / numpy 工具
# ============================================================

def read_wav_as_float32(path: Path):
    with wave.open(str(path), "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        frames = wf.readframes(wf.getnframes())

    if sample_width != 2:
        raise RuntimeError(f"只支持 16-bit PCM WAV，当前 sample_width={sample_width}")

    audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0

    if channels > 1:
        audio = audio.reshape(-1, channels).mean(axis=1)

    if sample_rate != TARGET_SAMPLE_RATE:
        audio = resample_linear(audio, sample_rate, TARGET_SAMPLE_RATE)

    return audio.astype(np.float32)


def write_wav_int16(path: Path, audio: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)

    audio = np.asarray(audio, dtype=np.float32)
    audio = np.clip(audio, -1.0, 1.0)

    pcm = (audio * 32767.0).astype(np.int16)

    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(TARGET_SAMPLE_RATE)
        wf.writeframes(pcm.tobytes())


def resample_linear(audio: np.ndarray, src_sr: int, dst_sr: int):
    if src_sr == dst_sr or len(audio) == 0:
        return audio.astype(np.float32)

    src_len = len(audio)
    dst_len = int(round(src_len * dst_sr / src_sr))

    if dst_len <= 1:
        return audio.astype(np.float32)

    src_x = np.linspace(0.0, 1.0, src_len)
    dst_x = np.linspace(0.0, 1.0, dst_len)

    return np.interp(dst_x, src_x, audio).astype(np.float32)


def change_speed(audio: np.ndarray, speed: float):
    if len(audio) < 2 or abs(speed - 1.0) < 0.01:
        return audio.astype(np.float32)

    new_len = max(2, int(round(len(audio) / speed)))

    src_x = np.linspace(0.0, 1.0, len(audio))
    dst_x = np.linspace(0.0, 1.0, new_len)

    return np.interp(dst_x, src_x, audio).astype(np.float32)


def db_to_gain(db: float):
    return 10.0 ** (db / 20.0)


def normalize_peak(audio: np.ndarray, target_peak_db: float):
    if len(audio) == 0:
        return audio.astype(np.float32)

    peak = float(np.max(np.abs(audio)))

    if peak < 1e-6:
        return audio.astype(np.float32)

    target_peak = db_to_gain(target_peak_db)
    audio = audio / peak * target_peak

    return np.clip(audio, -1.0, 1.0).astype(np.float32)


def add_random_silence(audio: np.ndarray):
    pre_ms = random.randint(50, 400)
    post_ms = random.randint(50, 500)

    pre = np.zeros(int(TARGET_SAMPLE_RATE * pre_ms / 1000), dtype=np.float32)
    post = np.zeros(int(TARGET_SAMPLE_RATE * post_ms / 1000), dtype=np.float32)

    return np.concatenate([pre, audio, post]).astype(np.float32)


def add_white_noise(audio: np.ndarray):
    if random.random() > 0.75:
        return audio.astype(np.float32)

    if len(audio) == 0:
        return audio.astype(np.float32)

    signal_rms = np.sqrt(np.mean(audio * audio)) + 1e-8

    # SNR 越大，噪声越小。这里不要太重，避免把 TTS 变得太假
    snr_db = random.uniform(16.0, 30.0)
    noise_rms = signal_rms / db_to_gain(snr_db)

    noise = np.random.normal(0.0, noise_rms, size=len(audio)).astype(np.float32)

    return np.clip(audio + noise, -1.0, 1.0).astype(np.float32)


def add_simple_echo(audio: np.ndarray):
    if random.random() > 0.30:
        return audio.astype(np.float32)

    delay_ms = random.randint(50, 130)
    delay_samples = int(TARGET_SAMPLE_RATE * delay_ms / 1000)

    echo_gain = db_to_gain(random.uniform(-20.0, -12.0))

    out = np.copy(audio).astype(np.float32)

    if 0 < delay_samples < len(audio):
        out[delay_samples:] += audio[:-delay_samples] * echo_gain

    return np.clip(out, -1.0, 1.0).astype(np.float32)


def augment_command_audio(audio: np.ndarray):
    audio = audio.astype(np.float32)

    # 较轻的速度变化，避免过度机械
    audio = change_speed(audio, random.uniform(0.94, 1.08))

    # 较轻的音量变化
    audio = audio * db_to_gain(random.uniform(-3.0, 2.0))

    # 前后静音
    audio = add_random_silence(audio)

    # 轻微回声
    audio = add_simple_echo(audio)

    # 轻到中等白噪声
    audio = add_white_noise(audio)

    # 峰值归一化，避免削顶
    audio = normalize_peak(audio, random.uniform(-6.0, -2.0))

    return audio.astype(np.float32)


def generate_silence_noise():
    duration_ms = random.randint(800, 2600)
    n = int(TARGET_SAMPLE_RATE * duration_ms / 1000)

    audio = np.zeros(n, dtype=np.float32)

    if random.random() < 0.75:
        noise_db = random.uniform(-42.0, -24.0)
        noise_amp = db_to_gain(noise_db)
        audio += np.random.normal(0.0, noise_amp, size=n).astype(np.float32)

    return np.clip(audio, -1.0, 1.0).astype(np.float32)


# ============================================================
# 4. edge-tts + imageio-ffmpeg
# ============================================================

async def get_available_chinese_voices():
    voices = await edge_tts.list_voices()

    all_zh_voices = []

    for v in voices:
        short_name = v.get("ShortName", "")
        locale = v.get("Locale", "")

        if any(locale.startswith(prefix) for prefix in VOICE_LOCALE_PREFIXES):
            all_zh_voices.append(short_name)

    all_zh_voices = sorted(set(all_zh_voices))

    if USE_ALL_CHINESE_VOICES:
        if all_zh_voices:
            return all_zh_voices
        raise RuntimeError("没有找到可用的中文 edge-tts 音色")

    preferred_available = [
        v for v in PREFERRED_VOICES
        if v in all_zh_voices
    ]

    if preferred_available:
        return preferred_available

    if all_zh_voices:
        return all_zh_voices

    raise RuntimeError("没有找到可用的中文 edge-tts 音色")


async def synthesize_to_mp3(text: str, voice: str, mp3_path: Path):
    rate = random.randint(-18, 18)
    volume = random.randint(-8, 8)

    communicate = edge_tts.Communicate(
        text=text,
        voice=voice,
        rate=f"{rate:+d}%",
        volume=f"{volume:+d}%",
    )

    await communicate.save(str(mp3_path))


def convert_mp3_to_wav(mp3_path: Path, wav_path: Path):
    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()

    cmd = [
        ffmpeg_exe,
        "-y",
        "-hide_banner",
        "-loglevel", "error",
        "-i", str(mp3_path),
        "-ac", "1",
        "-ar", str(TARGET_SAMPLE_RATE),
        "-sample_fmt", "s16",
        str(wav_path),
    ]

    subprocess.run(cmd, check=True)


def process_generated_mp3(tmp_mp3: Path, tmp_wav: Path, out_wav: Path):
    convert_mp3_to_wav(tmp_mp3, tmp_wav)
    audio = read_wav_as_float32(tmp_wav)
    audio = augment_command_audio(audio)
    write_wav_int16(out_wav, audio)


async def generate_one_tts_wav(text: str, voice: str, wav_path: Path):
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_mp3 = Path(tmpdir) / f"tts_{random.randint(100000, 999999)}.mp3"
        tmp_wav = Path(tmpdir) / f"tts_{random.randint(100000, 999999)}.wav"

        await synthesize_to_mp3(text, voice, tmp_mp3)

        # Windows 文件释放缓冲
        await asyncio.sleep(0.2)

        last_error = None

        for _ in range(8):
            try:
                await asyncio.to_thread(
                    process_generated_mp3,
                    tmp_mp3,
                    tmp_wav,
                    wav_path,
                )
                return
            except PermissionError as e:
                last_error = e
                await asyncio.sleep(0.25)
            except subprocess.CalledProcessError as e:
                last_error = e
                await asyncio.sleep(0.25)
            except Exception as e:
                last_error = e
                await asyncio.sleep(0.25)

        raise last_error


# ============================================================
# 5. 生成任务
# ============================================================

async def generate_command_item(semaphore, label: str, index: int, voices):
    async with semaphore:
        texts = COMMAND_TEXTS[label]
        text = random.choice(texts)
        voice = random.choice(voices)

        out_dir = OUTPUT_ROOT / label
        out_dir.mkdir(parents=True, exist_ok=True)

        out_file = out_dir / f"{label}_auto_{index:04d}.wav"

        print(f"[{label}] {index:04d} voice={voice} text={text}")

        try:
            await generate_one_tts_wav(text, voice, out_file)
            return True
        except Exception as e:
            print(f"  生成失败：{repr(e)}")
            return False


async def generate_unknown_item(semaphore, index: int, voices):
    async with semaphore:
        text = random.choice(UNKNOWN_TEXTS)
        voice = random.choice(voices)

        out_dir = OUTPUT_ROOT / "unknown"
        out_dir.mkdir(parents=True, exist_ok=True)

        out_file = out_dir / f"unknown_auto_{index:04d}.wav"

        print(f"[unknown] {index:04d} voice={voice} text={text}")

        try:
            await generate_one_tts_wav(text, voice, out_file)
            return True
        except Exception as e:
            print(f"  生成失败：{repr(e)}")
            return False


async def generate_silence_item(semaphore, index: int):
    async with semaphore:
        out_dir = OUTPUT_ROOT / "silence"
        out_dir.mkdir(parents=True, exist_ok=True)

        out_file = out_dir / f"silence_auto_{index:04d}.wav"

        print(f"[silence] {index:04d}")

        try:
            audio = generate_silence_noise()
            await asyncio.to_thread(write_wav_int16, out_file, audio)
            return True
        except Exception as e:
            print(f"  生成失败：{repr(e)}")
            return False


# ============================================================
# 6. 主流程
# ============================================================

def main():
    global OUTPUT_ROOT
    parser = argparse.ArgumentParser(description="生成待人工筛选的命令词合成数据")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args()
    OUTPUT_ROOT = args.output_dir
    asyncio.run(generate_dataset())


async def generate_dataset():
    global edge_tts, np, imageio_ffmpeg
    import edge_tts
    import numpy as np
    import imageio_ffmpeg
    if CLEAR_OLD_OUTPUT and OUTPUT_ROOT.exists():
        shutil.rmtree(OUTPUT_ROOT)

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    voices = await get_available_chinese_voices()

    print("可用中文音色数量：", len(voices))
    for v in voices:
        print("  ", v)

    print()
    print("输出目录：", OUTPUT_ROOT)
    print("并发数量：", CONCURRENCY)
    print()

    semaphore = asyncio.Semaphore(CONCURRENCY)

    tasks = []

    print("========== 创建命令音频任务 ==========")

    for label in COMMAND_TEXTS.keys():
        for i in range(1, NUM_PER_LABEL + 1):
            tasks.append(generate_command_item(semaphore, label, i, voices))

    print("========== 创建 unknown 音频任务 ==========")

    for i in range(1, NUM_UNKNOWN + 1):
        tasks.append(generate_unknown_item(semaphore, i, voices))

    print("========== 创建 silence 音频任务 ==========")

    for i in range(1, NUM_SILENCE + 1):
        tasks.append(generate_silence_item(semaphore, i))

    print()
    print(f"总任务数：{len(tasks)}")
    print("========== 开始生成 ==========")

    results = await asyncio.gather(*tasks)

    total_ok = sum(1 for r in results if r)
    total_fail = len(results) - total_ok

    print()
    print("========== 生成完成 ==========")
    print(f"输出目录：{OUTPUT_ROOT}")
    print(f"成功数量：{total_ok}")
    print(f"失败数量：{total_fail}")


if __name__ == "__main__":
    main()
