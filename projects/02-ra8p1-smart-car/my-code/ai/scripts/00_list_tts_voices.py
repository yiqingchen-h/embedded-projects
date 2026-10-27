"""列出 Windows 本地 TTS 语音。"""

from __future__ import annotations

import sys

from kws_common import configure_local_cache


def list_sapi_voices() -> int:
    configure_local_cache()
    try:
        import win32com.client
    except ImportError:
        print("未安装 pywin32，无法通过 SAPI 枚举语音。")
        return 1

    voice = win32com.client.Dispatch("SAPI.SpVoice")
    voices = voice.GetVoices()
    print(f"SAPI voices: {voices.Count}")
    for index in range(voices.Count):
        token = voices.Item(index)
        description = token.GetDescription()
        lang = token.GetAttribute("Language")
        gender = token.GetAttribute("Gender")
        print(f"[{index}] {description} | language={lang} | gender={gender} | id={token.Id}")
    return 0


def main() -> int:
    try:
        return list_sapi_voices()
    except Exception as exc:
        print(f"枚举 TTS 语音失败: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
