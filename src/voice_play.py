# 导入必要的库
from pydub import AudioSegment # 用于处理音频文件
from pydub.playback import play # 用于播放pydub处理后的音频
from playsound import playsound # 用于播放声音文件

# 直接播放"1.mp3"文件，这是一个简单的测试或示例播放
playsound("1.mp3")

# 加载音频文件"1.mp3"到AudioSegment对象
audio = AudioSegment.from_file("1.mp3")

# 播放AudioSegment对象中的音频
play(audio)

def play_voice(number):
    """
    根据传入的数字播放对应的MP3文件。

    参数:
        number (int): 音频文件的数字标识符（例如，如果传入1，则播放"1.mp3"）。
    """
    global times_voice_play # 声明使用全局变量times_voice_play，尽管在此片段中未定义或使用
    playsound(str(number)+".mp3") # 播放指定数字对应的MP3文件