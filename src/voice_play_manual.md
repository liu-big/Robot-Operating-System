# `voice_play.py` 脚本使用开发操作手册

## 1. 概述

`voice_play.py` 是一个简单的 Python 脚本，用于播放音频文件。它利用 `pydub` 和 `playsound` 库来实现音频的加载和播放功能。该脚本主要用于在机器人应用中提供语音反馈或提示，例如播放预设的语音指令、状态通知或警告音。

## 2. 脚本结构

```
src/
├── voice_play.py             # 主脚本文件
└── voice_play_manual.md      # 本文档
```

### 2.1 脚本内容解析

`voice_play.py` 脚本的核心功能是播放 `.mp3` 格式的音频文件。它包含了两种播放音频的方式：

- **直接播放**：
  ```python
  playsound("1.mp3")
  ```
  这行代码直接使用 `playsound` 库播放名为 `1.mp3` 的音频文件。这种方式简单直接，适用于快速播放单个音频。

- **使用 `pydub` 加载和播放**：
  ```python
  audio = AudioSegment.from_file("1.mp3")
  play(audio)
  ```
  这两行代码首先使用 `pydub.AudioSegment.from_file()` 加载 `1.mp3` 文件到内存中，然后使用 `pydub.playback.play()` 函数播放加载的音频。`pydub` 提供了更强大的音频处理能力，例如剪辑、合并、调整音量等，尽管在这个脚本中只使用了基本的播放功能。

- **`play_voice(number)` 函数**：
  ```python
  def play_voice(number):
      global times_voice_play
      playsound(str(number)+".mp3")
  ```
  这个函数接受一个数字作为参数，并将其转换为字符串，然后拼接成一个文件名（例如 `"1.mp3"`、`"2.mp3"` 等），最后使用 `playsound` 播放对应的音频文件。这使得脚本能够根据传入的数字动态播放不同的音频文件，非常适合需要播放一系列编号音频的场景。

## 3. 主要功能与用途

- **音频播放**：提供简单易用的音频播放功能。
- **语音反馈**：在机器人操作中提供语音提示、警告或状态通知。
- **模块化设计**：`play_voice` 函数使得音频播放功能可以方便地集成到其他 Python 脚本中。

## 4. 使用方法

### 4.1 依赖

确保您的 Python 环境中安装了以下必要的库：

- `pydub`
- `playsound`

您可以使用 `pip` 安装它们：

```bash
pip install pydub playsound
```

`pydub` 依赖于 `ffmpeg` 或 `libav`。您可能需要根据您的操作系统安装这些多媒体处理工具。例如，在 Ubuntu 上：

```bash
sudo apt-get install ffmpeg
```

### 4.2 音频文件准备

将您需要播放的 `.mp3` 音频文件放置在与 `voice_play.py` 脚本相同的目录下，或者在脚本中指定音频文件的完整路径。

### 4.3 运行脚本

1. **直接运行**：
   ```bash
   python voice_play.py
   ```
   这将播放脚本中硬编码的 `1.mp3` 文件。

2. **调用 `play_voice` 函数**：
   您可以在其他 Python 脚本中导入 `voice_play` 模块，并调用 `play_voice` 函数来播放指定的音频文件：
   ```python
   import voice_play

   # 播放 2.mp3
   voice_play.play_voice(2)

   # 播放 5.mp3
   voice_play.play_voice(5)
   ```

## 5. 项目全局应用

在 `ucar_ws` 项目中，`voice_play.py` 脚本可以用于：

- **导航提示**：在机器人到达某个导航点时播放语音提示。
- **任务状态通知**：例如，在完成某个任务步骤时播放“任务完成”的语音。
- **错误警告**：在机器人遇到障碍物或发生故障时播放警告音。
- **人机交互**：通过语音反馈增强机器人与用户之间的交互体验。

## 6. 维护与更新

- **音频文件管理**：定期检查音频文件是否存在，并确保其命名符合 `play_voice` 函数的预期（例如 `1.mp3`, `2.mp3`）。
- **库版本更新**：保持 `pydub` 和 `playsound` 库的更新，以获取最新的功能和 bug 修复。
- **错误处理**：可以添加 `try-except` 块来处理文件不存在或播放失败的情况，提高脚本的健壮性。

## 7. 故障排除

- **音频无法播放**：
  - **原因**：音频文件路径错误，音频文件损坏，或者 `ffmpeg` / `libav` 未安装或配置不正确。
  - **解决方案**：
    - 检查音频文件路径是否正确。
    - 尝试使用其他播放器播放音频文件，确认文件本身没有问题。
    - 确保 `ffmpeg` 或 `libav` 已正确安装，并且其可执行文件在系统的 PATH 环境变量中。
- **`playsound` 报错**：
  - **原因**：`playsound` 库可能依赖于特定的系统播放器。
  - **解决方案**：
    - 确保系统上安装了默认的音频播放器。
    - 检查 `playsound` 的官方文档，了解其在不同操作系统上的依赖。
- **`pydub` 报错**：
  - **原因**：通常是 `ffmpeg` 或 `libav` 相关的问题。
  - **解决方案**：
    - 确保 `ffmpeg` 或 `libav` 已正确安装。
    - 检查 `pydub` 的官方文档，了解其对 `ffmpeg` 的版本要求和配置方法。