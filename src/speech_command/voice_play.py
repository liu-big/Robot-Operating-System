# -*- coding: utf-8 -*-
"""
@file voice_play.py
@brief 语音播放脚本。

该脚本用于加载并播放音频文件，以及根据传入的数字播放对应的语音文件。
主要使用了 pydub 库进行音频处理和播放。
"""

from pydub import AudioSegment
from pydub.playback import play

# 加载音频文件
audio = AudioSegment.from_file("spontoon2f.mp3")

# 播放音频
play(audio)


def play_voice(number):
    """
    根据传入的数字播放对应的语音文件。

    Args:
        number (int): 语音文件的数字标识，例如 1 对应 1.mp3。

    Returns:
        None
    """
    global times_voice_play
    playsound(str(number)+".mp3")