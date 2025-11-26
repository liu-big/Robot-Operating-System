#ifndef AIUIAGENTTESTER_H_
#define AIUIAGENTTESTER_H_

#include <aiui/AIUI.h>
#include <FileUtil.h>
#include <WriteAudioThread.h>
#include <AudioRecorder.h>
#include <TestListener.h>
#include <AudioPlayer.h>
#include <msp_cmn.h>
#include <iostream>
#include <Global.h>
#include "jsoncpp/json/json.h"
#include <iostream>
#include <RingBuffer.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include <string>
//
#include "alsa/asoundlib.h"
#include "stdio.h"
#include "stdlib.h"
#include <numeric>
#include "signal.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

typedef void (*TEST_CALLBACK)(); 
static TEST_CALLBACK testCallback;

//
enum class MsgType : unsigned char {
    Shake = 0x01,
    WIFI_SETTING = 0x02,
    AIUI_SETTING = 0x03,
    AIUI_MSG = 0x04,
    CONTROL = 0x05,
    CUSTOM_MGS = 0x2A,
    CONFIRM = 0xff
};
struct MsgPacket
{
    unsigned char uid;
    unsigned char type;

    unsigned short size;
    unsigned short sid;

    string bytes;
};


class AIUITester
{
private:
	IAIUIAgent *agent; //aiui代理
	TestListener listener;
	AudioRecorder *audioRecorder; //与录音相关的
	AudioPlayer *audioPlayer;	  //与音频播放相关
public:
	AIUITester();
	~AIUITester();
    void destory();
private:
	void showIntroduction(bool detail);
	//创建AIUI 代理，通过AIUI代理与SDK发送消息通信
	void createAgent();
	//唤醒接口
	void wakeup();

	//开始AIUI，调用stop()之后需要调用此接口才可以与AIUI继续交互
	void start();
	//停止AIUI
	void stop();
	//usb麦克风设备创建
	void recorder_creat();
	//麦克风录音开始
	void recorder_start();
	//麦克风录音结束
	void recorder_stop();
	void stopWriteThread();
	void reset();
	
	void buildGrammar();
	void updateLocalLexicon();

public:
	void readCmd();
	void test();
	void bind(TEST_CALLBACK callback);
};
void gTTS(string text);
#endif /* AIUIAGENTTESTER_H_ */
