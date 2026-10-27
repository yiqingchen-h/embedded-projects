"""从原始 3 秒 WAV 中剪出 1 秒有效语音，输出 16kHz 单声道 PCM16 WAV。

三种使用方式，按需选一种：

方式 1 - VAD 直接剪辑（最简单，不需要模型）
    python scripts/07_trim_dataset_audio.py

方式 2 - VAD + KWS 模型选最佳候选（模型练准后用）
    python scripts/07_trim_dataset_audio.py --use-model

方式 3 - VAD + Whisper 过滤无效录音（模型不准时用）
    python scripts/07_trim_dataset_audio.py --write-all-candidates --whisper-score --whisper-device GPU.1

文件顶部开关:
    USE_MODEL_SCORING = False  设 True 后默认启用方式 2

常用参数:
    --input-dir / --output-dir    指定输入输出目录
    --candidate-index 0~4         手动选候选，默认 2(中间)
    --write-all-candidates        导出全部 5 个候选
    --transcribe-mode speech      宽松（默认，有命令词就处理）
    --transcribe-mode strict      严格（标签必须完全匹配）
    --model-path                  指定 TFLite 模型路径
"""

from __future__ import annotations

import argparse
import sys
import wave
from dataclasses import dataclass
from pathlib import Path

AI_ROOT = Path(__file__).resolve().parents[1]


# ── Whisper 评分（可选，需要 openvino_genai）──
try:
    from batch_transcribe_GPU import LABEL_TEXT_MAP, COMMAND_KEYWORDS, INITIAL_PROMPT, HOTWORDS, check_openvino_device
except ImportError:
    LABEL_TEXT_MAP = {}
    COMMAND_KEYWORDS = []
    INITIAL_PROMPT = ""
    HOTWORDS = ""



DEFAULT_WHISPER_MODEL_DIR = AI_ROOT / "whisper-medium-openvino"


class WhisperScorer:
    """用 OpenVINO Whisper 转录候选窗口，根据 LABEL_TEXT_MAP 匹配期望标签。

    使用方式：
        scorer = WhisperScorer(model_dir, LABEL_TEXT_MAP, device="GPU.1")
        text, score = scorer.score_candidate(wav_path, expected_label)
    """

    def __init__(self, model_dir: Path, label_map: dict[str, list[str]],
                 device: str | None = None, initial_prompt: str = "", hotwords: str = ""):
        check_openvino_device(device)
        import openvino_genai
        if not model_dir.exists():
            raise FileNotFoundError(f"Whisper 模型目录不存在: {model_dir}")
        print(f"[WhisperScorer] 加载模型: {model_dir} device={device}")
        self.pipe = openvino_genai.WhisperPipeline(str(model_dir), device)
        self.label_map = label_map
        self.initial_prompt = initial_prompt
        self.hotwords = hotwords
        self.all_keywords: list[str] = []
        for texts in label_map.values():
            self.all_keywords.extend(texts)

    def transcribe(self, wav_path: Path) -> str:
        """转录 WAV，返回去空白的小写文本。"""
        sample_rate, pcm = wavfile.read(str(wav_path))
        # 确保 float32 单声道 16kHz
        if pcm.ndim > 1:
            pcm = pcm.astype(np.float32).mean(axis=1)
        audio = pcm.astype(np.float32) / 32767.0
        if sample_rate != 16000:
            audio = resample_linear(audio, sample_rate, 16000)
        result = self.pipe.generate(
            audio,
            language="<|zh|>",
            initial_prompt=self.initial_prompt,
            hotwords=self.hotwords,
            max_new_tokens=64)
        return str(result).strip().lower()

    def score_candidate(self, wav_path: Path, expected_label: str) -> tuple[str, float]:
        """转录一个候选 WAV，返回 (转录文本, 匹配分数 0.0~1.0)。

        严格匹配：只检查 LABEL_TEXT_MAP 中本类显式列出的关键词。
        不跨类匹配，不模糊兜底，不在表中的一律 0 分。

        分数规则：
            1.0 = 命中本类最长关键词（4字+）
            0.8 = 命中本类中等关键词（3字）
            0.6 = 命中本类最短关键词（1-2字）
            0.0 = 未命中本类任何关键词（包括空白、噪声、误识别）
        """
        text = self.transcribe(wav_path)
        expected_texts = self.label_map.get(expected_label, [])

        # 层级正向匹配：长词优先
        for kw in expected_texts:
            if kw in text:
                kw_len = len(kw)
                if kw_len >= 4:
                    return text, 1.0
                elif kw_len >= 3:
                    return text, 0.8
                else:
                    return text, 0.6

        # 不在本类关键词表中 → 0 分，不做跨类兜底
        return text, 0.0



# 默认输入为采集脚本保存的原始数据，默认输出为独立 trimmed 目录，避免覆盖原始 WAV。
DEFAULT_INPUT_DIR = AI_ROOT / "dataset_real" / "raw"
#DEFAULT_INPUT_DIR = AI_ROOT / "dataset_real" / "auto"

DEFAULT_OUTPUT_DIR = AI_ROOT / "dataset_real" / "trimmed"
#DEFAULT_OUTPUT_DIR = AI_ROOT / "dataset_real" / "auto_trimmed"
# 模型输入采样率，必须与 MCU 端 VOICE_AI_MODEL_SAMPLE_RATE_HZ 保持一致。
MODEL_SAMPLE_RATE_HZ = 16000

# 默认模型输入长度，对应 1 秒音频。
DEFAULT_TARGET_SECONDS = 1.0

# 语音有效段搜索窗口长度，单位秒，对应 MCU 端 VOICE_AI_SEARCH_SECONDS。
DEFAULT_SEARCH_SECONDS = 3.0

# 滑窗候选数量：0%、25%、50%、75%、100% 五个对齐位置。
VOICE_AI_MAX_CANDIDATES = 5

# VAD 使用 10ms 帧，16kHz 下 160 点。
VOICE_AI_VAD_FRAME_SAMPLES = 160

# 固定最低 VAD 阈值，float 音频幅值。
# 之前 0.020 太高，噪声底 0.001、语音幅值 0.01~0.02 的安静录音会被整段跳过。
# 降为 0.005（约 -46dBFS），低于此值视为静音。
VOICE_AI_VAD_FIXED_THRESHOLD = 0.005

# 自适应阈值倍率：threshold=max(fixed, noise_floor*ratio)。
# 噪声底 ~0.001 × 4.0 = 0.004，与 fixed 0.005 形成合理衔接。
VOICE_AI_VAD_NOISE_RATIO = 4.0

# 连续超过阈值的帧数，达到后认为进入语音段。
VOICE_AI_VAD_START_FRAMES = 3

# 连续低于阈值的帧数，达到后认为语音结束。
VOICE_AI_VAD_END_SILENCE_FRAMES = 20

# 小于该帧数的 active 段认为是毛刺/风噪。
VOICE_AI_VAD_MIN_ACTIVE_FRAMES = 8

# 裁剪时保留语音起点前 200ms，必须与 MCU 端 VOICE_AI_VAD_PRE_ROLL_FRAMES 一致。
VOICE_AI_VAD_PRE_ROLL_FRAMES = 20

# 裁剪时保留语音终点后 50ms，必须与 MCU 端 VOICE_AI_VAD_POST_ROLL_FRAMES 一致。
VOICE_AI_VAD_POST_ROLL_FRAMES = 5

# 没有 VAD active 时是否仍然使用最近 1 秒。0 表示静音/风噪跳过。
VOICE_AI_NO_ACTIVE_FALLBACK = 0

# ═══════════════════════════════════════════════════════════════════════
# 当前支持的模型推理配置
# ═══════════════════════════════════════════════════════════════════════

# 是否启用模型评分自动选候选。模型初次训练不准确时保持 False，回退到 --candidate-index。
USE_MODEL_SCORING = False

# float 模型路径。
TFLITE_MODEL_PATH = AI_ROOT / "artifacts_real" / "model_float.tflite"

# 标签文件路径。
LABELS_PATH = AI_ROOT / "labels.txt"

# 模型评分使用所选模型配套清单中的归一化参数。

# 峰值归一化目标。
TARGET_PEAK = 0.75

# 预加重系数。
PRE_EMPHASIS = 0.97

# STFT 参数。
FRAME_LENGTH = 400
FRAME_STEP = 160
N_FFT = 512

# Mel 滤波器参数。
MEL_BINS = 40
LOW_HZ = 20.0
HIGH_HZ = 7600.0


@dataclass(frozen=True)
class CandidatePlan:
    """和 MCU 端 voice_ai_candidate_plan_t 对齐的候选窗口计划。"""

    found_active: bool
    active_start_sample: int
    active_end_sample: int
    candidate_start_samples: tuple[int, ...]
    noise_floor: float
    threshold: float


@dataclass(frozen=True)
class TrimResult:
    """单个 WAV 的剪辑结果。"""

    status: str
    message: str
    output_path: Path | None = None
    start_sample: int = 0
    end_sample: int = 0
    candidate_index: int = 0
    active_start_sample: int = 0
    active_end_sample: int = 0
    noise_floor: float = 0.0
    threshold: float = 0.0
    # 模型评分相关字段
    expected_label: str = ""
    top1_label: str = ""
    model_score: float = 0.0
    model_match: bool = False


def read_wav_pcm16(path: Path) -> tuple[int, np.ndarray]:
    """读取 WAV 并转换为单声道 PCM16。

    训练数据最终统一按单声道处理；如果输入是双声道，这里先取平均值。
    """

    sample_rate, data = wavfile.read(str(path))
    if data.ndim > 1:
        data = data.astype(np.float32).mean(axis=1)

    if np.issubdtype(data.dtype, np.floating):
        data = np.clip(data, -1.0, 1.0)
        pcm = (data * 32767.0).astype(np.int16)
    elif data.dtype == np.int16:
        pcm = data
    elif np.issubdtype(data.dtype, np.integer):
        max_value = float(np.iinfo(data.dtype).max)
        pcm = np.clip(data.astype(np.float32) / max_value, -1.0, 1.0)
        pcm = (pcm * 32767.0).astype(np.int16)
    else:
        raise RuntimeError(f"不支持的 WAV 数据类型: {data.dtype}")

    return int(sample_rate), np.asarray(pcm, dtype=np.int16)


def write_wav_pcm16(path: Path, sample_rate: int, pcm: np.ndarray) -> None:
    """写出单声道 PCM16 WAV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    pcm = np.asarray(pcm, dtype=np.int16)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm.tobytes())


def pcm16_to_float(pcm: np.ndarray) -> np.ndarray:
    """把 PCM16 转换为 MCU 端一致的 float 幅值范围。"""

    return np.asarray(pcm, dtype=np.float32) / 32767.0


def float_to_pcm16(audio: np.ndarray) -> np.ndarray:
    """把 float 音频转换回 PCM16。"""

    clipped = np.clip(np.asarray(audio, dtype=np.float32), -1.0, 1.0)
    positive = clipped >= 0.0

    out = np.empty(clipped.shape, dtype=np.int16)
    out[positive] = (clipped[positive] * 32767.0 + 0.5).astype(np.int16)
    out[~positive] = (clipped[~positive] * 32768.0 - 0.5).astype(np.int16)
    return out


def resample_linear(audio: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """使用线性插值重采样，对齐 MCU 端 voice_ai_resample_to_model_rate() 思路。"""

    audio = np.asarray(audio, dtype=np.float32)

    if src_rate <= 0:
        raise RuntimeError(f"采样率非法: {src_rate}")

    if src_rate == dst_rate:
        return audio.copy()

    if audio.size < 2:
        return np.zeros(0, dtype=np.float32)

    duration = audio.size / float(src_rate)
    dst_samples = int(round(duration * dst_rate))
    if dst_samples < 2:
        return np.zeros(0, dtype=np.float32)

    src_positions = np.linspace(0.0, float(audio.size - 1), num=dst_samples, dtype=np.float32)
    left = np.floor(src_positions).astype(np.int64)
    right = np.minimum(left + 1, audio.size - 1)
    frac = src_positions - left.astype(np.float32)

    return audio[left] + ((audio[right] - audio[left]) * frac)


def select_search_window(audio: np.ndarray, search_samples: int, align: str) -> tuple[np.ndarray, int]:
    """从输入音频中选出 3 秒搜索窗口。"""

    if search_samples <= 0:
        raise RuntimeError("search_samples 必须大于 0")

    if audio.size == search_samples:
        return audio.copy(), 0

    if audio.size < search_samples:
        output = np.zeros(search_samples, dtype=np.float32)
        output[:audio.size] = audio
        return output, 0

    if align == "start":
        start = 0
    elif align == "center":
        start = (audio.size - search_samples) // 2
    elif align == "end":
        start = audio.size - search_samples
    else:
        raise RuntimeError(f"不支持的搜索窗口对齐方式: {align}")

    end = start + search_samples
    return audio[start:end].copy(), start


def clamp_window_start(start_sample: int, src_samples: int, window_samples: int) -> int:
    """将窗口起点限制到搜索窗口内，对齐 MCU 端 voice_ai_clamp_window_start()。"""

    max_start = 0
    if src_samples > window_samples:
        max_start = src_samples - window_samples

    if start_sample <= 0:
        return 0

    if start_sample > max_start:
        return max_start

    return int(start_sample)


def add_candidate(candidates: list[int], start_sample: int, src_samples: int, window_samples: int) -> None:
    """向候选计划中追加一个窗口起点。"""

    if len(candidates) >= VOICE_AI_MAX_CANDIDATES:
        return

    candidates.append(clamp_window_start(start_sample, src_samples, window_samples))


def compute_vad_energy(audio: np.ndarray, frame_samples: int) -> tuple[np.ndarray, float]:
    """计算 10ms 帧平均绝对幅值并估计噪声底，对齐 MCU 端 voice_ai_compute_vad_energy()。"""

    if audio.size == 0:
        return np.zeros(0, dtype=np.float32), 0.0

    frame_count = audio.size // frame_samples
    if frame_count <= 0:
        return np.zeros(0, dtype=np.float32), 0.0

    trimmed = audio[:frame_count * frame_samples]
    frames = trimmed.reshape(frame_count, frame_samples)
    energy = np.mean(np.abs(frames), axis=1).astype(np.float32)

    mean_all = float(np.mean(energy))
    low_values = energy[energy <= mean_all]
    if low_values.size > 0:
        noise_floor = float(np.mean(low_values))
    else:
        noise_floor = mean_all

    return energy, noise_floor


def build_candidate_plan(
    audio: np.ndarray,
    target_samples: int,
    fixed_threshold: float,
    noise_ratio: float,
    start_frames: int,
    end_silence_frames: int,
    min_active_frames: int,
    pre_roll_frames: int,
    post_roll_frames: int,
    no_active_fallback: int,
) -> CandidatePlan | None:
    """用帧级 VAD 从 3 秒搜索窗口构造 5 个 1 秒候选窗口。"""

    samples = int(audio.size)
    if samples < target_samples:
        return None

    vad_energy, noise_floor = compute_vad_energy(audio, VOICE_AI_VAD_FRAME_SAMPLES)
    frame_count = int(vad_energy.size)
    if frame_count == 0:
        return None

    threshold = noise_floor * noise_ratio
    if threshold < fixed_threshold:
        threshold = fixed_threshold

    in_active = False
    selected = False
    above_count = 0
    silence_count = 0
    segment_start_frame = 0
    last_active_frame = 0
    selected_start_frame = 0
    selected_end_frame = 0

    for frame in range(frame_count):
        active = bool(vad_energy[frame] > threshold)

        if not in_active:
            if active:
                above_count += 1
                if above_count >= start_frames:
                    in_active = True
                    segment_start_frame = frame + 1 - start_frames
                    last_active_frame = frame
                    silence_count = 0
            else:
                above_count = 0
        else:
            if active:
                last_active_frame = frame
                silence_count = 0
            else:
                silence_count += 1
                if silence_count >= end_silence_frames:
                    segment_end_frame = last_active_frame + 1
                    if (
                        segment_end_frame > segment_start_frame
                        and (segment_end_frame - segment_start_frame) >= min_active_frames
                    ):
                        selected = True
                        selected_start_frame = segment_start_frame
                        selected_end_frame = segment_end_frame

                    in_active = False
                    above_count = 0
                    silence_count = 0

    if in_active:
        segment_end_frame = last_active_frame + 1
        if (
            segment_end_frame > segment_start_frame
            and (segment_end_frame - segment_start_frame) >= min_active_frames
        ):
            selected = True
            selected_start_frame = segment_start_frame
            selected_end_frame = segment_end_frame

    if not selected:
        if no_active_fallback:
            candidates: list[int] = []
            add_candidate(candidates, samples - target_samples, samples, target_samples)
            if not candidates:
                return None
            return CandidatePlan(
                found_active=False,
                active_start_sample=0,
                active_end_sample=0,
                candidate_start_samples=tuple(candidates),
                noise_floor=noise_floor,
                threshold=threshold,
            )

        return None

    if selected_start_frame > pre_roll_frames:
        selected_start_frame -= pre_roll_frames
    else:
        selected_start_frame = 0

    selected_end_frame += post_roll_frames
    if selected_end_frame > frame_count:
        selected_end_frame = frame_count

    active_start_sample = selected_start_frame * VOICE_AI_VAD_FRAME_SAMPLES
    active_end_sample = selected_end_frame * VOICE_AI_VAD_FRAME_SAMPLES
    if active_end_sample > samples:
        active_end_sample = samples

    if active_end_sample <= active_start_sample:
        return None

    candidates = []
    active_samples = active_end_sample - active_start_sample

    if active_samples >= target_samples:
        align_range = active_samples - target_samples

        for candidate in range(VOICE_AI_MAX_CANDIDATES):
            offset = (align_range * candidate) // (VOICE_AI_MAX_CANDIDATES - 1)
            start_sample = active_start_sample + offset
            add_candidate(candidates, start_sample, samples, target_samples)
    else:
        align_range = target_samples - active_samples

        for candidate in range(VOICE_AI_MAX_CANDIDATES):
            offset = (align_range * candidate) // (VOICE_AI_MAX_CANDIDATES - 1)
            start_sample = active_start_sample - offset
            add_candidate(candidates, start_sample, samples, target_samples)

    if not candidates:
        return None

    return CandidatePlan(
        found_active=True,
        active_start_sample=active_start_sample,
        active_end_sample=active_end_sample,
        candidate_start_samples=tuple(candidates),
        noise_floor=noise_floor,
        threshold=threshold,
    )


def copy_candidate_to_audio(audio: np.ndarray, start_sample: int, target_samples: int) -> np.ndarray:
    """从搜索窗口复制 1 秒候选音频到输出缓冲，对齐 MCU 端 voice_ai_copy_candidate_to_audio()。"""

    output = np.zeros(target_samples, dtype=np.float32)

    if start_sample >= audio.size:
        return output

    copy_samples = audio.size - start_sample
    if copy_samples > target_samples:
        copy_samples = target_samples

    if copy_samples > 0:
        output[:copy_samples] = audio[start_sample:start_sample + copy_samples]

    return output


def candidate_output_path(output_path: Path, candidate_index: int) -> Path:
    """为多候选导出生成独立文件名。"""

    return output_path.with_name(f"{output_path.stem}_c{candidate_index}{output_path.suffix}")


def load_labels() -> list[str]:
    """从 labels.txt 读取标签列表。"""

    labels = []
    for line in LABELS_PATH.read_text(encoding="utf-8").splitlines():
        label = line.strip()
        if label and not label.startswith("#"):
            labels.append(label)
    if not labels:
        raise RuntimeError(f"标签文件为空: {LABELS_PATH}")
    return labels


def hz_to_mel(hz):
    """Hz 转 mel 刻度。"""

    return 2595.0 * np.log10(1.0 + np.asarray(hz) / 700.0)


def mel_to_hz(mel):
    """mel 刻度转 Hz。"""

    return 700.0 * (10.0 ** (np.asarray(mel) / 2595.0) - 1.0)


def make_mel_filterbank() -> np.ndarray:
    """生成当前配置的 40×257 mel 滤波器组。"""

    low_mel = hz_to_mel(LOW_HZ)
    high_mel = hz_to_mel(HIGH_HZ)
    mel_points = np.linspace(low_mel, high_mel, MEL_BINS + 2)
    hz_points = mel_to_hz(mel_points)
    bins = np.floor((N_FFT + 1) * hz_points / MODEL_SAMPLE_RATE_HZ).astype(int)
    bins = np.clip(bins, 0, N_FFT // 2)

    filters = np.zeros((MEL_BINS, N_FFT // 2 + 1), dtype=np.float32)
    for m in range(1, MEL_BINS + 1):
        left = bins[m - 1]
        center = bins[m]
        right = bins[m + 1]
        if center <= left:
            center = left + 1
        if right <= center:
            right = center + 1
        right = min(right, N_FFT // 2)
        for k in range(left, min(center, filters.shape[1])):
            filters[m - 1, k] = (k - left) / max(center - left, 1)
        for k in range(center, min(right, filters.shape[1])):
            filters[m - 1, k] = (right - k) / max(right - center, 1)
    return filters


def preprocess_for_model(audio: np.ndarray, feature_mean: float, feature_std: float) -> np.ndarray:
    """1 秒 16kHz float 音频 → (1, 98, 40, 1) 模型输入特征。

    使用当前支持的 log-mel 流程及模型清单中的归一化参数。
    """

    audio = audio.astype(np.float32)

    # 1. 补齐或截断到 16000 采样点
    if audio.size < MODEL_SAMPLE_RATE_HZ:
        audio = np.pad(audio, (0, MODEL_SAMPLE_RATE_HZ - audio.size))
    elif audio.size > MODEL_SAMPLE_RATE_HZ:
        audio = audio[:MODEL_SAMPLE_RATE_HZ]

    # 2. 峰值归一化到 TARGET_PEAK
    peak = float(np.max(np.abs(audio)))
    if peak > 1e-6:
        audio = audio * (TARGET_PEAK / peak)

    # 3. 预加重
    audio = signal.lfilter([1.0, -PRE_EMPHASIS], [1.0], audio).astype(np.float32)

    # 4. 分帧 + Hann 窗
    frame_count = 1 + (len(audio) - FRAME_LENGTH) // FRAME_STEP
    frames = np.lib.stride_tricks.sliding_window_view(audio, FRAME_LENGTH)[::FRAME_STEP][:frame_count]
    window = np.hanning(FRAME_LENGTH).astype(np.float32)
    framed = frames * window

    # 5. FFT → 功率谱
    spectrum = np.fft.rfft(framed, n=N_FFT)
    power = (np.abs(spectrum) ** 2).astype(np.float32)

    # 6. Mel 滤波 → log
    mel_filters = make_mel_filterbank()
    mel = np.maximum(power @ mel_filters.T, 1e-8)
    log_mel = np.log(mel).astype(np.float32)

    # 7. Z-score 标准化
    log_mel = (log_mel - feature_mean) / feature_std

    # 8. 扩展为 (1, 98, 40, 1)
    return log_mel.reshape(1, frame_count, MEL_BINS, 1)


class KwsScorer:
    """加载 TFLite 模型并对 1 秒 16kHz float 音频推理，返回按模型清单标签顺序对齐的分数。"""

    def __init__(self, model_path: Path, manifest_path: Path | None = None):
        from kws_common import load_model_manifest, validate_model_tensors
        self.manifest = load_model_manifest(model_path, manifest_path)
        import tensorflow as tf

        if not model_path.exists():
            raise FileNotFoundError(f"模型文件不存在: {model_path}")

        self.labels = list(self.manifest["labels"])
        self._interpreter = tf.lite.Interpreter(model_content=model_path.read_bytes())
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        validate_model_tensors(self._input_details, self._output_details, self.manifest)

    @property
    def label_count(self) -> int:
        return len(self.labels)

    def score(self, audio_16k: np.ndarray) -> np.ndarray:
        """对 1 秒 16kHz float 音频推理，返回按模型清单标签顺序对齐的 softmax 分数。"""

        features = preprocess_for_model(audio_16k, self.manifest["feature_mean"], self.manifest["feature_std"])
        self._interpreter.set_tensor(self._input_details[0]["index"], features)
        self._interpreter.invoke()
        output = self._interpreter.get_tensor(self._output_details[0]["index"])
        return np.asarray(output[0], dtype=np.float32)

    def score_for_label(self, audio_16k: np.ndarray, label_index: int) -> float:
        """对指定标签返回分数。"""

        scores = self.score(audio_16k)
        if label_index < 0 or label_index >= len(scores):
            return 0.0
        return float(scores[label_index])


def trim_one_file(
    input_path: Path,
    output_path: Path,
    target_seconds: float,
    search_seconds: float,
    search_align: str,
    candidate_index: int,
    write_all_candidates: bool,
    fixed_threshold: float,
    noise_ratio: float,
    start_frames: int,
    end_silence_frames: int,
    min_active_frames: int,
    pre_roll_frames: int,
    post_roll_frames: int,
    no_active_fallback: int,
    scorer: KwsScorer | None = None,
    whisper_scorer: WhisperScorer | None = None,
) -> list[TrimResult]:
    """剪辑单个 WAV 文件。"""

    source_sample_rate, pcm = read_wav_pcm16(input_path)

    target_samples = int(round(MODEL_SAMPLE_RATE_HZ * target_seconds))
    if target_samples <= 0:
        return [TrimResult("skip", "--target-seconds 必须大于 0")]

    search_samples = int(round(MODEL_SAMPLE_RATE_HZ * search_seconds))
    if search_samples < target_samples:
        return [TrimResult("skip", "--search-seconds 不能小于 --target-seconds")]

    if candidate_index < 0 or candidate_index >= VOICE_AI_MAX_CANDIDATES:
        return [TrimResult("skip", f"--candidate-index 必须在 0~{VOICE_AI_MAX_CANDIDATES - 1} 之间")]

    # 确定期望标签（从目录结构 raw/wake/xxx.wav → wake）
    expected_label = input_path.parent.name

    # silence/unknown 没有语音是正常的，无 VAD 命中时回退到最后 1 秒
    actual_no_active_fallback = no_active_fallback
    if expected_label in ("silence", "unknown"):
        actual_no_active_fallback = 1

    audio = pcm16_to_float(pcm)
    audio_16k = resample_linear(audio, source_sample_rate, MODEL_SAMPLE_RATE_HZ)
    search_audio, search_offset = select_search_window(audio_16k, search_samples, search_align)

    plan = build_candidate_plan(
        search_audio,
        target_samples,
        fixed_threshold,
        noise_ratio,
        start_frames,
        end_silence_frames,
        min_active_frames,
        pre_roll_frames,
        post_roll_frames,
        actual_no_active_fallback,
    )
    if plan is None:
        return [TrimResult("skip", "没有找到 MCU VAD 有效语音段")]
    expected_label_index = -1
    if scorer is not None:
        for i, name in enumerate(scorer.labels):
            if name == expected_label:
                expected_label_index = i
                break

    # silence 和 unknown 属于干扰音/静音，模型评分无意义，强制回退到 candidate-index
    use_model = scorer is not None and expected_label not in ("silence", "unknown")

    selected_indices: list[int] = []

    if use_model:
        # 用模型对 5 个候选分别推理，选 top-1 分数最高的候选
        best_index = 0
        best_score = -1.0
        best_top1_label = ""
        best_top1_idx = 0
        candidate_scores: list[tuple[int, float, int, float]] = []

        for idx in range(len(plan.candidate_start_samples)):
            start = int(plan.candidate_start_samples[idx])
            candidate_audio = copy_candidate_to_audio(search_audio, start, target_samples)
            scores = scorer.score(candidate_audio)
            top1_idx = int(np.argmax(scores))
            top1_score = float(scores[top1_idx])
            expected_score = float(scores[expected_label_index]) if expected_label_index >= 0 else 0.0
            candidate_scores.append((idx, top1_score, top1_idx, expected_score))

            if top1_score > best_score:
                best_score = top1_score
                best_index = idx
                best_top1_idx = top1_idx

        best_top1_label = scorer.labels[best_top1_idx] if best_top1_idx < scorer.label_count else "?"
        best_model_score = best_score
        best_model_match = (best_top1_label == expected_label)

        # 打印每个候选的得分
        for idx, top1_score, top1_idx, expected_score in candidate_scores:
            top1_label = scorer.labels[top1_idx] if top1_idx < scorer.label_count else "?"
            tag = " <-- best" if idx == best_index else ""
            print(f"  cand={idx} top1={top1_label}({top1_score:.4f}) expect={expected_label}({expected_score:.4f}){tag}")

        selected_indices = [best_index]
    elif whisper_scorer is not None and expected_label not in ("silence", "unknown"):
        # ── Whisper 评分模式：5 候选各跑一次 Whisper，选转录最匹配标签的 ──
        whisper_best_index = candidate_index
        whisper_best_score = -1.0
        whisper_best_text = ""
        max_candidates = min(len(plan.candidate_start_samples), 5)
        import tempfile as _tmpfile_mod, os as _os
        for idx in range(max_candidates):
            start = int(plan.candidate_start_samples[idx])
            cand_audio = copy_candidate_to_audio(search_audio, start, target_samples)
            with _tmpfile_mod.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp_path = Path(tmp.name)
            write_wav_pcm16(tmp_path, MODEL_SAMPLE_RATE_HZ, float_to_pcm16(cand_audio))
            text, score = whisper_scorer.score_candidate(tmp_path, expected_label)
            _os.unlink(tmp_path)
            if score > whisper_best_score:
                whisper_best_score = score
                whisper_best_index = idx
                whisper_best_text = text
        if whisper_best_score > 0.0:
            selected_indices = [whisper_best_index]
            best_top1_label = whisper_best_text
            best_model_score = whisper_best_score
            best_model_match = (whisper_best_score >= 1.0)
            print(f"  W={whisper_best_index} text='{whisper_best_text}' score={whisper_best_score:.1f}")
        else:
            print(f"  W=all0 -> fallback cand={candidate_index}")
            selected_indices = [candidate_index]
            best_top1_label = ""
            best_model_score = 0.0
            best_model_match = False
    else:
        # Whisper 回退：不选
        pass

    # 如果还没确定 selected_indices，按 write_all_candidates 或 fallback 处理
    if not selected_indices:
        if write_all_candidates:
            selected_indices = list(range(len(plan.candidate_start_samples)))
            best_top1_label = ""
            best_model_score = 0.0
            best_model_match = False
        else:
            # silence/unknown 无 VAD 命中时只生成 1 个 fallback 候选，强制用第 0 个
            actual_index = 0 if not plan.found_active else candidate_index
            if actual_index >= len(plan.candidate_start_samples):
                return [TrimResult("skip", f"候选数量不足: got={len(plan.candidate_start_samples)} need_index={actual_index}")]
            selected_indices = [actual_index]
            best_top1_label = ""
            best_model_score = 0.0
            best_model_match = False

    results: list[TrimResult] = []

    for idx in selected_indices:
        start = int(plan.candidate_start_samples[idx])
        candidate_audio = copy_candidate_to_audio(search_audio, start, target_samples)
        candidate_pcm = float_to_pcm16(candidate_audio)

        if write_all_candidates:
            current_output_path = candidate_output_path(output_path, idx)
        else:
            current_output_path = output_path

        write_wav_pcm16(current_output_path, MODEL_SAMPLE_RATE_HZ, candidate_pcm)

        results.append(
            TrimResult(
                status="ok",
                message="完成",
                output_path=current_output_path,
                start_sample=search_offset + start,
                end_sample=search_offset + start + target_samples,
                candidate_index=idx,
                active_start_sample=search_offset + plan.active_start_sample,
                active_end_sample=search_offset + plan.active_end_sample,
                noise_floor=plan.noise_floor,
                threshold=plan.threshold,
                expected_label=expected_label,
                top1_label=best_top1_label,
                model_score=best_model_score,
                model_match=best_model_match,
            )
        )

    return results


def iter_wav_files(input_dir: Path):
    """递归遍历输入目录中的 WAV，保留标签子目录结构。"""

    for path in sorted(input_dir.rglob("*.wav")):
        if path.is_file():
            yield path


def load_transcribe_csv(csv_path: Path) -> dict[str, dict[str, str]]:
    """从 batch_transcribe_GPU.py 输出的 CSV 构建相对路径 → 行数据 的映射。"""

    import csv

    data: dict[str, dict[str, str]] = {}
    with csv_path.open("r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rel = row.get("relative_path", "").strip()
            if rel:
                data[rel] = row
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description="批量剪辑 WAV，使用 MCU 端 VAD 候选窗口方法")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR, help="输入 WAV 根目录")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="输出 WAV 根目录")
    parser.add_argument("--target-seconds", type=float, default=DEFAULT_TARGET_SECONDS, help="输出音频长度，单位秒")
    parser.add_argument("--search-seconds", type=float, default=DEFAULT_SEARCH_SECONDS, help="搜索窗口长度，单位秒")
    parser.add_argument("--search-align", choices=("start", "center", "end"), default="start", help="输入音频长于搜索窗口时的截取位置")
    parser.add_argument("--candidate-index", type=int, default=2, help="不导出全部候选时使用的候选序号，默认 2")
    parser.add_argument("--write-all-candidates", action="store_true", help="导出 5 个候选窗口，文件名追加 _c0~_c4")
    parser.add_argument("--vad-fixed-threshold", type=float, default=VOICE_AI_VAD_FIXED_THRESHOLD, help="固定最低 VAD 阈值，float 幅值")
    parser.add_argument("--vad-noise-ratio", type=float, default=VOICE_AI_VAD_NOISE_RATIO, help="自适应噪声底倍率")
    parser.add_argument("--vad-start-frames", type=int, default=VOICE_AI_VAD_START_FRAMES, help="连续多少帧超过阈值后认为语音开始")
    parser.add_argument("--vad-end-silence-frames", type=int, default=VOICE_AI_VAD_END_SILENCE_FRAMES, help="连续多少帧低于阈值后认为语音结束")
    parser.add_argument("--vad-min-active-frames", type=int, default=VOICE_AI_VAD_MIN_ACTIVE_FRAMES, help="最小有效语音帧数")
    parser.add_argument("--vad-pre-roll-frames", type=int, default=VOICE_AI_VAD_PRE_ROLL_FRAMES, help="语音开始前保留帧数")
    parser.add_argument("--vad-post-roll-frames", type=int, default=VOICE_AI_VAD_POST_ROLL_FRAMES, help="语音结束后保留帧数")
    parser.add_argument("--no-active-fallback", type=int, default=VOICE_AI_NO_ACTIVE_FALLBACK, choices=(0, 1), help="未检测到 active 时是否回退到最后 1 秒")
    parser.add_argument("--use-model", action="store_true", help="启用模型评分自动选候选（覆盖文件顶部 USE_MODEL_SCORING）")
    parser.add_argument("--model-path", type=Path, default=TFLITE_MODEL_PATH, help="TFLite 模型路径")
    parser.add_argument("--model-manifest", type=Path, default=None, help="默认读取模型同目录的 kws_model_manifest.json")
    parser.add_argument("--transcribe-csv", type=Path, default=None, help="batch_transcribe_GPU.py 输出的 CSV，用于 ASR 辅助剪辑")
    parser.add_argument("--transcribe-mode", choices=("strict", "speech"), default="speech", help="strict=标签完全匹配才处理 speech=检测到任意命令词即处理（默认 speech，容忍口音）")
    parser.add_argument("--whisper-score", action="store_true", help="5 个候选各跑一次 Whisper，选转录最匹配标签的候选")
    parser.add_argument("--whisper-model", type=Path, default=DEFAULT_WHISPER_MODEL_DIR, help="Whisper OpenVINO 模型目录")
    parser.add_argument("--whisper-device", type=str, default=None, help="Whisper 推理设备 (GPU.1 / CPU)")
    parser.add_argument("--whisper-max-candidates", type=int, default=5, help="最多对前 N 个候选跑 Whisper，加速用")
    args = parser.parse_args()
    if args.whisper_score and not args.whisper_device:
        parser.error("--whisper-score 需要显式提供 --whisper-device")
    global np, signal, wavfile
    import numpy as np
    from scipy import signal
    from scipy.io import wavfile

    if not args.input_dir.exists():
        print(f"输入目录不存在: {args.input_dir}")
        return 1

    if args.target_seconds <= 0:
        print("--target-seconds 必须大于 0")
        return 1

    if args.search_seconds <= 0:
        print("--search-seconds 必须大于 0")
        return 1

    if args.search_seconds < args.target_seconds:
        print("--search-seconds 不能小于 --target-seconds")
        return 1

    # 加载模型评分器
    use_model = args.use_model or USE_MODEL_SCORING
    scorer = None
    if use_model:
        try:
            scorer = KwsScorer(args.model_path, args.model_manifest)
            print(f"模型已加载: {args.model_path}")
            print(f"模型选候选已启用（标签数={scorer.label_count}）")
        except Exception as exc:
            print(f"模型评分配置无效: {exc}")
            return 1
    else:
        print(f"模型选候选未启用，使用 --candidate-index={args.candidate_index}")

    # 加载 ASR 转录 CSV
    asr_csv: dict[str, dict[str, str]] | None = None
    if args.transcribe_csv is not None:
        if not args.transcribe_csv.exists():
            print(f"转录 CSV 不存在: {args.transcribe_csv}")
            return 1
        asr_csv = load_transcribe_csv(args.transcribe_csv)
        print(f"ASR 辅助剪辑已启用: {args.transcribe_csv}")
        print(f"CSV 条目数: {len(asr_csv)}")
        print(f"剪辑模式: {args.transcribe_mode}（strict=标签完全匹配 speech=检测到语音即处理）")

    # Whisper 评分模式
    whisper_scorer = None
    if args.whisper_score:
        whisper_scorer = WhisperScorer(args.whisper_model, LABEL_TEXT_MAP,
                                        device=args.whisper_device,
                                        initial_prompt=INITIAL_PROMPT,
                                        hotwords=HOTWORDS)
        print(f"Whisper 评分已启用: {args.whisper_model} device={args.whisper_device}")

    print()

    total = 0
    ok_model_match = 0
    ok = 0
    skipped = 0
    skipped_by_asr = 0

    # 预计算总文件数
    all_files = list(iter_wav_files(args.input_dir))
    grand_total = len(all_files)
    print(f"待处理文件总数: {grand_total}")
    print()

    for input_path in all_files:
        total += 1
        relative_path = input_path.relative_to(args.input_dir)
        # 输出路径沿用相对路径，因此 raw/wake/a.wav 会变成 trimmed/wake/a.wav。
        output_path = args.output_dir / relative_path

        # ASR 辅助判断
        if asr_csv is not None:
            # silence/unknown 本就没有命令词，ASR 检测不到命令词才是正确的，直接放行
            expected_label = input_path.parent.name
            is_noise_label = expected_label in ("silence", "unknown")

            if not is_noise_label:
                row = asr_csv.get(str(relative_path))
                if row is None:
                    skipped_by_asr += 1
                    print(f"[{total}/{grand_total}] ASR-SKIP {relative_path}: CSV 中无此文件记录")
                    continue

                command_hits = row.get("command_hits", "").strip()
                pass_by_label = row.get("pass_by_label", "").strip()

                if args.transcribe_mode == "strict":
                    if pass_by_label != "1":
                        skipped_by_asr += 1
                        print(f"[{total}/{grand_total}] ASR-SKIP {relative_path}: 标签不匹配（pass_by_label={pass_by_label}）")
                        continue
                else:
                    # speech 模式：检测到任意命令词即处理，容忍口音导致的对不上
                    if not command_hits:
                        skipped_by_asr += 1
                        print(f"[{total}/{grand_total}] ASR-SKIP {relative_path}: 未检测到命令词")
                        continue

        try:
            results = trim_one_file(
                input_path=input_path,
                output_path=output_path,
                target_seconds=args.target_seconds,
                search_seconds=args.search_seconds,
                search_align=args.search_align,
                candidate_index=args.candidate_index,
                write_all_candidates=args.write_all_candidates,
                fixed_threshold=args.vad_fixed_threshold,
                noise_ratio=args.vad_noise_ratio,
                start_frames=args.vad_start_frames,
                end_silence_frames=args.vad_end_silence_frames,
                min_active_frames=args.vad_min_active_frames,
                pre_roll_frames=args.vad_pre_roll_frames,
                post_roll_frames=args.vad_post_roll_frames,
                no_active_fallback=args.no_active_fallback,
                scorer=scorer,
                whisper_scorer=whisper_scorer,
            )
        except Exception as exc:
            skipped += 1
            print(f"[{total}/{grand_total}] SKIP {relative_path}: {exc}")
            continue

        file_ok = False
        for result in results:
            if result.status == "ok":
                file_ok = True
                ok += 1
                model_info = ""
                if scorer is not None:
                    match_tag = "MATCH" if result.model_match else "MISMATCH"
                    model_info = f" model={result.top1_label}({result.model_score:.4f}) {match_tag}"
                    if result.model_match:
                        ok_model_match += 1
                asr_info = ""
                if asr_csv is not None:
                    row = asr_csv.get(str(relative_path))
                    if row is not None:
                        asr_text = row.get("asr_text", "").strip()
                        if asr_text:
                            asr_info = f' asr="{asr_text}"'
                print(
                    f"[{total}/{grand_total}] OK   "
                    f"{relative_path} -> {result.output_path} "
                    f"cand={result.candidate_index} "
                    f"start={result.start_sample / MODEL_SAMPLE_RATE_HZ:.3f}s "
                    f"active={result.active_start_sample / MODEL_SAMPLE_RATE_HZ:.3f}s~{result.active_end_sample / MODEL_SAMPLE_RATE_HZ:.3f}s "
                    f"noise={result.noise_floor:.5f} th={result.threshold:.5f}"
                    f"{model_info}"
                    f"{asr_info}"
                )
            else:
                print(f"[{total}/{grand_total}] SKIP {relative_path}: {result.message}")

        if not file_ok:
            skipped += 1

    if total == 0:
        print(f"没有找到 WAV: {args.input_dir}")
        return 1

    print(f"完成: total={total} ok={ok} skipped={skipped}")
    if asr_csv is not None:
        print(f"ASR 过滤: {skipped_by_asr} 个文件未通过转录验证")
    if scorer is not None and ok > 0:
        match_rate = ok_model_match / ok * 100.0 if ok > 0 else 0.0
        print(f"模型匹配: {ok_model_match}/{ok} ({match_rate:.1f}%)")
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
