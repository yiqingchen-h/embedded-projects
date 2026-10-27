"""对指定目录的一条音频进行三轮 OpenVINO Whisper 推理计时。"""
import argparse
import time
from pathlib import Path

AI_DIR = Path(__file__).resolve().parents[1]
RAW_DIR = AI_DIR / "dataset_real" / "raw"
MODEL_DIR = AI_DIR / "whisper-base-openvino"

INITIAL_PROMPT = (
    "这是一个智能小车语音命令识别任务。"
    "可能出现的语音包括：你好小车，开始，停止，前进，后退，"
    "左平移，右平移，左转，右转，抓取方块，抓取圆柱，抓取圆球，"
    "抓河马，抓小猫，抓石墩子，抓水豚，抓玉米，抓蕉太狼，抓小狗，抓TNT。"
)


def find_first_audio(audio_dir: Path):
    files = []
    for ext in ["*.wav", "*.mp3", "*.m4a", "*.flac", "*.aac", "*.ogg"]:
        files.extend(audio_dir.glob(ext))

    files = sorted(files)

    if not files:
        raise FileNotFoundError(f"没有找到音频文件：{audio_dir}")

    return files[0]


def read_audio_16k(audio_file: Path):
    import librosa
    audio, sr = librosa.load(str(audio_file), sr=16000)
    return audio.tolist()



def main():
    parser = argparse.ArgumentParser(description="OpenVINO Whisper 单音频推理检查")
    parser.add_argument("--input-dir", type=Path, default=RAW_DIR / "wake")
    parser.add_argument("--model-dir", type=Path, default=MODEL_DIR)
    parser.add_argument("--device", required=True, help="明确指定 OpenVINO 设备，例如 GPU.1 或 CPU")
    args = parser.parse_args()
    from batch_transcribe_GPU import check_openvino_device
    check_openvino_device(args.device)
    if not args.model_dir.is_dir():
        parser.error(f"模型目录不存在: {args.model_dir}")
    audio_file = find_first_audio(args.input_dir)
    import openvino_genai
    print(f"模型目录：{args.model_dir}")
    print(f"推理设备：{args.device}")
    print(f"测试音频：{audio_file}")
    t0 = time.time()
    pipe = openvino_genai.WhisperPipeline(str(args.model_dir), args.device)
    print(f"模型加载耗时：{time.time() - t0:.3f} 秒")
    raw_speech = read_audio_16k(audio_file)
    for i in range(3):
        t0 = time.time()
        result = pipe.generate(raw_speech, language="<|zh|>",
                               initial_prompt=INITIAL_PROMPT, max_new_tokens=64)
        print(f"第 {i + 1} 次推理耗时：{time.time() - t0:.3f} 秒")
        print(f"识别结果：{result}")


if __name__ == "__main__":
    main()
