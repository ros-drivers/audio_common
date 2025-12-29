"""
Piper TTS Plugin - ROS1 style synchronous implementation
"""

import os
import tempfile
import subprocess

try:
    from sound_play.sound_play_plugin import SoundPlayPlugin
except ImportError:
    from sound_play_plugin import SoundPlayPlugin


class PiperTTSPlugin(SoundPlayPlugin):
    def __init__(self):
        super(PiperTTSPlugin, self).__init__()
        
        # Piper 配置（可通过 ROS 参数覆盖）
        self.piper_executable = '/usr/local/bin/piper'
        self.model_path = '/opt/piper/models/zh_CN-huayan-medium.onnx'
        self.timeout = 10
        
        # 验证可用性
        self._check_availability()
    
    def _check_availability(self):
        if not os.path.exists(self.piper_executable):
            print(f"[PiperTTS] 错误: Piper 未找到: {self.piper_executable}")
            return
        
        if not os.path.exists(self.model_path):
            print(f"[PiperTTS] 错误: 模型未找到: {self.model_path}")
            return
        
        print(f"[PiperTTS] 初始化成功")
        print(f"[PiperTTS] 可执行文件: {self.piper_executable}")
        print(f"[PiperTTS] 模型: {self.model_path}")
    
    def sound_play_say_plugin(self, text, voice):
        fd, wavfilename = tempfile.mkstemp(
            suffix='.wav',
            prefix='piper_tts_'
        )
        os.close(fd)
        
        try:
            # 调用 Piper
            process = subprocess.Popen(
                [
                    self.piper_executable,
                    '--model', self.model_path,
                    '--output_file', wavfilename
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # 发送文本并等待完成
            stdout, stderr = process.communicate(
                input=text,
                timeout=self.timeout
            )
            
            # 检查结果
            if process.returncode != 0:
                print(f"[PiperTTS] 合成失败: {stderr}")
                if os.path.exists(wavfilename):
                    os.remove(wavfilename)
                return None
            
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
            
        except subprocess.TimeoutExpired:
            print(f"[PiperTTS] 合成超时")
            process.kill()
            if os.path.exists(wavfilename):
                os.remove(wavfilename)
            return None
        except Exception as e:
            print(f"[PiperTTS] 异常: {e}")
            if os.path.exists(wavfilename):
                os.remove(wavfilename)
            return None
