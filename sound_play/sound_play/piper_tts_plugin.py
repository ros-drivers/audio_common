"""
Piper TTS Plugin - Using piper-tts Python API
"""

import os
import tempfile
import wave

try:
    from piper import PiperVoice
except ImportError:
    print("[PiperTTS] 错误: 需要安装 piper-tts: pip install piper-tts")
    PiperVoice = None

try:
    from sound_play.sound_play_plugin import SoundPlayPlugin
except ImportError:
    from sound_play_plugin import SoundPlayPlugin


class PiperTTSPlugin(SoundPlayPlugin):
    # 音频格式常量
    AUDIO_CHANNELS = 1  # 单声道
    AUDIO_SAMPLE_WIDTH = 2  # 16-bit (2 bytes)
    
    def __init__(self):
        super(PiperTTSPlugin, self).__init__()
        
        # Piper 配置（可通过 ROS 参数覆盖）
        self.model_path = '/opt/piper/models/zh_CN-huayan-medium.onnx'
        self.config_path = self.model_path + '.json'
        self.voice = None
        
        # 验证可用性并加载模型
        self._check_availability()
    
    def _check_availability(self):
        if PiperVoice is None:
            print(f"[PiperTTS] 错误: piper-tts 库未安装")
            return
        
        if not os.path.exists(self.model_path):
            print(f"[PiperTTS] 错误: 模型未找到: {self.model_path}")
            return
        
        if not os.path.exists(self.config_path):
            print(f"[PiperTTS] 警告: 配置文件未找到: {self.config_path}")
        
        try:
            # 加载 Piper 语音模型
            self.voice = PiperVoice.load(self.model_path, config_path=self.config_path, use_cuda=False)
            print(f"[PiperTTS] 初始化成功")
            print(f"[PiperTTS] 模型: {self.model_path}")
        except Exception as e:
            print(f"[PiperTTS] 加载模型失败: {e}")
            self.voice = None
    
    def sound_play_say_plugin(self, text, voice):
        if self.voice is None:
            print(f"[PiperTTS] 错误: 语音模型未加载")
            return None

        # 创建临时文件
        fd, wavfilename = tempfile.mkstemp(
            suffix='.wav',
            prefix='piper_tts_'
        )
        os.close(fd)

        try:
            # Piper API 兼容 bytes 或 generator
            audio_data = self.voice.synthesize(text)

            audio_chunks = []

            if isinstance(audio_data, (bytes, bytearray)):
                audio_chunks.append(audio_data)
            else:
                for chunk in audio_data:
                    # Piper 新版：AudioChunk
                    if hasattr(chunk, "samples"):
                        audio_chunks.append(chunk.samples.tobytes())

                    # 中间版：array / memoryview / numpy
                    elif hasattr(chunk, "tobytes"):
                        audio_chunks.append(chunk.tobytes())

                    # 兜底（极少发生）
                    else:
                        audio_chunks.append(bytes(chunk))

            audio_bytes = b"".join(audio_chunks)


            # 写入 WAV 文件
            with wave.open(wavfilename, 'wb') as wav_file:
                wav_file.setnchannels(self.AUDIO_CHANNELS)
                wav_file.setsampwidth(self.AUDIO_SAMPLE_WIDTH)
                wav_file.setframerate(self.voice.config.sample_rate)
                wav_file.writeframes(audio_bytes)

            # 验证文件
            if not os.path.exists(wavfilename):
                print(f"[PiperTTS] 输出文件未生成")
                return None

            if os.path.getsize(wavfilename) == 0:
                print(f"[PiperTTS] 输出文件为空")
                os.remove(wavfilename)
                return None

            print(f"[PiperTTS] 合成成功: {wavfilename}")
            return wavfilename

        except Exception as e:
            print(f"[PiperTTS] 异常: {e}")
            if os.path.exists(wavfilename):
                os.remove(wavfilename)
            return None

