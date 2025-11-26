/**
 * @file AudioPlayer.cpp
 * @brief 音频播放器实现文件。
 * 该文件包含了音频播放器的功能实现，主要用于通过 ALSA 库播放音频数据。
 */

#include <AudioPlayer.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <unistd.h>

// 静态成员变量初始化
char *AudioPlayer::device_name = nullptr; /**< @brief 音频设备名称。 */

/**
 * @brief AudioPlayer 类的构造函数。
 * 初始化 ALSA 音频设备，设置播放参数，如采样率、声道数、格式等。
 */
AudioPlayer::AudioPlayer()
{
	device_name = (char*)"default"; // "plughw:0,0" 或 "default"，用于指定播放设备
	sample_rat = 16000; // 采样率
	channel_num = 1;    // 声道数

	int err;
	// 打开 PCM 设备进行播放
	if ((err = snd_pcm_open(&handle, device_name, SND_PCM_STREAM_PLAYBACK, 0)) < 0)
	{
		fprintf(stderr, "无法打开音频设备 %s (%s)\n",
				device_name,
				snd_strerror(err));
		exit(1);
	}

	// 分配硬件参数结构体
	if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0)
	{
		fprintf(stderr, "无法分配硬件参数结构体 (%s)\n",
				snd_strerror(err));	
		exit(1);
	}

	// 初始化硬件参数结构体
	if ((err = snd_pcm_hw_params_any(handle, hw_params)) < 0)
	{
		fprintf(stderr, "无法初始化硬件参数结构体 (%s)\n",
				snd_strerror(err));
		exit(1);
	}

	// 设置访问类型为交错模式读写
	if ((err = snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
	{
		fprintf(stderr, "无法设置访问类型 (%s)\n",
				snd_strerror(err));
		exit(1);
	}

	// 设置采样格式为 16 位小端有符号整数
	if ((err = snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0)
	{
		fprintf(stderr, "无法设置采样格式 (%s)\n",
				snd_strerror(err));
		exit(1);
	}
	// 设置采样率
	if ((err = snd_pcm_hw_params_set_rate_near(handle, hw_params, &sample_rat, 0)) < 0)
	{
		fprintf(stderr, "无法设置采样率 (%s)\n",
				snd_strerror(err));
		exit(1);
	}

	// 设置声道数
	if ((err = snd_pcm_hw_params_set_channels(handle, hw_params, channel_num)) < 0)
	{
		fprintf(stderr, "无法设置声道数 (%s)\n",
				snd_strerror(err));
		exit(1);
	}

	// 将硬件参数写入驱动
	if ((err = snd_pcm_hw_params(handle, hw_params)) < 0)
	{
		fprintf(stderr, "无法设置参数 (%s)\n",
				snd_strerror(err));
		exit(1);
	}

	snd_pcm_hw_params_free(hw_params); // 释放硬件参数结构体

	// 准备音频接口以供使用
	if ((err = snd_pcm_prepare(handle)) < 0)
	{
		fprintf(stderr, "无法准备音频接口 (%s)\n",
				snd_strerror(err));
		exit(1);
	}
	snd_pcm_hw_params_get_period_size(hw_params, &uframes, 0); // 获取周期大小
}

/**
 * @brief 写入音频数据到播放设备。
 * @param buf 包含音频数据的缓冲区。
 */
void AudioPlayer::Write(unsigned char *buf)
{
	int err, cptr;
	cptr = 512;
	while (cptr > 0)
	{
		err = snd_pcm_writei(handle, buf, cptr);
		usleep(10000); // 等待 10 毫秒
		if (err == -EAGAIN)
			continue;
		if (err < 0)
		{
			std::cout << "跳过一个周期" << std::endl;
			break; /* skip one period */
		}
		buf += err * 1;
		cptr -= err;
	}
}

/**
 * @brief 处理 ALSA 播放器的欠载/过载错误恢复。
 * @param handle PCM 设备句柄。
 * @param err 错误码。
 * @return 成功返回 0，否则返回错误码。
 */
int xrun_recovery(snd_pcm_t *handle, int err)
{
	if (err == -EPIPE)
	{
		/* 欠载 (under-run) */
		err = snd_pcm_prepare(handle); // 准备 PCM 设备

		if (err < 0)
			printf("无法从欠载中恢复，准备失败: %s\n",
				   snd_strerror(err));
		return 0;
	}
	else if (err == -ESTRPIPE)
	{
		// 尝试从挂起状态恢复
		while ((err = snd_pcm_resume(handle)) == -EAGAIN)
			usleep(1000); // 等待 1 毫秒，直到挂起标志被释放
		if (err < 0)
		{
			err = snd_pcm_prepare(handle); // 准备 PCM 设备
			if (err < 0)
				printf("无法从挂起中恢复，准备失败: %s\n",
					   snd_strerror(err));
		}
		return 0;
	}
	return err;
}

/**
 * @brief 写入指定长度的音频数据到播放设备。
 * @param buf 包含音频数据的缓冲区。
 * @param buf_len 缓冲区中音频数据的字节长度。
 */
void AudioPlayer::Write(unsigned char *buf, int buf_len)
{
	int err;
	// 写入音频数据，buf_len / 2 是因为 snd_pcm_writei 期望的是帧数，而不是字节数
	err = snd_pcm_writei(handle, buf, buf_len / 2); 
	err = xrun_recovery(handle, err); // 处理可能的欠载/过载错误
	if (err < 0)
	{
		printf("写入错误: %s\n", snd_strerror(err));
	}
}

/**
 * @brief 清除播放缓冲区并准备音频设备。
 */
void AudioPlayer::Clear_Write()
{
	// snd_pcm_drop(handle); // 丢弃所有未播放的帧
	// snd_pcm_drain(handle); // 等待所有帧播放完毕
	// snd_pcm_close(handle); // 关闭 PCM 设备
	snd_pcm_prepare(handle); // 准备音频设备
}

/**
 * @brief 测试写入音频数据。
 * 写入固定大小的样本数据到播放设备。
 */
void AudioPlayer::WriteTest()
{
	unsigned char samples[1280];
	unsigned char *ptr;
	int err, cptr;
	ptr = samples;
	cptr = 1280;
	while (cptr > 0)
	{
		err = snd_pcm_writei(handle, ptr, cptr);
		if (err == -EAGAIN)
			continue;
		if (err < 0)
		{
			std::cout << "snd_pcm_writei() 错误码 < 0" << std::endl;
			break;
		}
		ptr += err * 1;
		cptr -= err;
	}
}

/**
 * @brief AudioPlayer 类的析构函数。
 * 关闭 ALSA PCM 设备。
 */
AudioPlayer::~AudioPlayer()
{
	snd_pcm_close(handle);
}

/**
 * @brief 测试函数。
 * 打印测试信息并循环。
 */
void AudioPlayer::Test()
{
	printf("AudioPlayer::Test()\n");
	unsigned char buffer[1024];
	for (int i = 0; i < 160; i++)
	{
		int rc;
		rc = snd_pcm_writei(handle, buffer, sizeof(buffer));
		if (rc < 0)
			rc = snd_pcm_recover(handle, rc, 0);
		if (rc < 0)
		{
			printf("snd_pcm_writei failed: %s\n", snd_strerror(rc));
			break;
		}
		if (rc > 0 && rc < (long)sizeof(buffer))
			printf("Short write (expected %li, wrote %li)\n", (long)sizeof(buffer), rc);
		sleep(2);
	}
}
