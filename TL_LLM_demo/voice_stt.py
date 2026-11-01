#!/usr/bin/env python3
# coding=utf-8
"""
语音输入模块（本地识别, 录音不上传）
====================================
· record_utterance(): 麦克风录音, 回车后开始, 说完自动在停顿处停止(动态噪声阈值)
· SttEngine: faster-whisper 中文识别(首次使用自动从 hf-mirror 下载模型)

依赖: pip install faster-whisper sounddevice
"""
import os
import time

# 国内镜像: 模型从 hf-mirror 下载(必须在 huggingface_hub 导入前设置)
os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")

# Anaconda 的 MKL(numpy) 和 ctranslate2 各带一份 OpenMP 运行库, 直接混用会报
# "OMP: Error #15"; 这里先加载 ctranslate2 并允许重复, 规避该问题
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
try:
    import ctranslate2  # noqa: F401  (必须先于 numpy/MKL 加载)
except Exception:
    pass

SAMPLE_RATE = 16000


def list_mics():
    """列出可用的麦克风设备"""
    import sounddevice as sd
    print("输入设备列表(可用 device 序号):")
    found = False
    for i, d in enumerate(sd.query_devices()):
        if d.get("max_input_channels", 0) > 0:
            found = True
            mark = "  <-- 默认" if i == sd.default.device[0] else ""
            print(f"  [{i}] {d['name']}{mark}")
    if not found:
        print("  没找到任何输入设备! 检查系统麦克风设置")


def record_utterance(max_sec=8.0, silence_sec=0.9, device=None, blocksize_sec=0.1):
    """
    录音直到静音。返回 (音频float32数组, None) 或 (None, 错误说明)。
    先测 0.4s 环境噪声 -> 动态阈值 = max(0.012, 噪声*3)
    """
    import numpy as np
    import sounddevice as sd
    block = int(SAMPLE_RATE * blocksize_sec)
    try:
        stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                                dtype="float32", blocksize=block, device=device)
    except Exception as e:
        return None, f"打不开麦克风: {e} (用 --init-voice 列出设备检查)"

    chunks = []
    with stream:
        # 环境噪声标定
        ambient = []
        t0 = time.time()
        while time.time() - t0 < 0.4:
            data, _ = stream.read(block)
            ambient.append(float(np.sqrt(np.mean(data ** 2))))
        noise = sum(ambient) / max(len(ambient), 1)
        threshold = max(0.012, noise * 3.0)
        print(f"🎤 录音中... (环境噪声 {noise:.4f}) —— 说完停顿一下自动结束")

        started = False
        silent = 0.0
        t0 = time.time()
        while True:
            data, _ = stream.read(block)
            chunks.append(data.copy())
            rms = float(np.sqrt(np.mean(data ** 2)))
            if rms > threshold:
                started = True
                silent = 0.0
            elif started:
                silent += blocksize_sec
            if started and silent >= silence_sec:
                break
            if time.time() - t0 >= max_sec:
                break

    if not started:
        return None, "没有检测到说话声(靠麦克风近一点再说一次)"
    audio = np.concatenate(chunks).flatten()
    return audio, None


class SttEngine:
    """faster-whisper 识别引擎(中文)"""

    DEFAULT_PROMPT = ("机械臂指令。向上抬五十毫米，向下走一点点，画个正方形，"
                      "回原点，速度慢一点，停下急停，张开夹爪，合上夹爪，现在在哪。")

    def __init__(self, model_size="small", device="cpu", language="zh",
                 download_root=None, initial_prompt=None):
        if device == "cuda":
            _add_cuda_dlls()
        from faster_whisper import WhisperModel
        self.language = language
        self.initial_prompt = initial_prompt or self.DEFAULT_PROMPT
        print(f"⏳ 加载语音模型 {model_size} ({device}) (首次运行会从 hf-mirror 下载) ...")
        t0 = time.time()
        compute_type = "int8" if device == "cpu" else "float16"
        self.model = WhisperModel(model_size, device=device,
                                  compute_type=compute_type,
                                  download_root=download_root)
        print(f"✅ 语音模型就绪 ({time.time() - t0:.1f}s)")

    def transcribe(self, audio):
        """audio: 16k 单声道 float32 -> 识别文本"""
        segments, _info = self.model.transcribe(
            audio,
            language=self.language,
            beam_size=3,
            vad_filter=True,
            condition_on_previous_text=False,
            initial_prompt=self.initial_prompt,
        )
        return "".join(s.text for s in segments).strip()


def _add_cuda_dlls():
    """让 ctranslate2 找到 pip 安装的 cuBLAS/cuDNN(GPU模式用)"""
    try:
        import nvidia.cublas
        import nvidia.cudnn
        for mod in (nvidia.cublas, nvidia.cudnn):
            bin_dir = os.path.join(os.path.dirname(mod.__file__), "bin")
            if os.path.isdir(bin_dir):
                os.add_dll_directory(bin_dir)
    except Exception:
        pass
