class SoundPlayPlugin(object):
    """sound_play 插件基类
    
    所有 TTS 后端（Piper, Festival, eSpeak 等）必须继承此类
    并实现 sound_play_say_plugin 方法
    
    插件定义格式（YAML）：
    - name: sound_play/piper_plugin
      module: sound_play.piper_tts_plugin.PiperTTSPlugin
    
    导出方式（package.xml）：
    <export>
      <sound_play plugin="${prefix}/sound_play_plugin.yaml" />
    </export>
    """
    
    def __init__(self):
        pass
    
    def sound_play_say_plugin(self, text, voice):
        """合成语音的插件方法
        
        Args:
            text (str): 要合成的文本
            voice (str): 语音名称（可选）
        """
        return None
