/**
 * @file AudioRecorder.cpp
 * @brief 音频录制器实现文件。
 * 该文件包含了音频录制器的功能实现，用于开始/停止录制降噪和原始音频，以及设置录音角度。
 */

#include <AudioRecorder.h>
#include <iostream>

using namespace std;

// 静态成员变量初始化
bool AudioRecorder::if_success_boot = false; /**< @brief 标记是否成功启动。 */
bool AudioRecorder::if_awake = false; /**< @brief 标记是否处于唤醒状态。 */
hid_device *AudioRecorder::handle = nullptr; /**< @brief HID 设备句柄。 */

/**
 * @brief AudioRecorder 类的构造函数。
 * @param audioPath 音频文件保存路径。
 */
AudioRecorder::AudioRecorder(const string& audioPath)
{

}

/**
 * @brief AudioRecorder 类的析构函数。
 * 负责销毁 AIUI 代理并停止所有录音操作。
 */
AudioRecorder::~AudioRecorder()
{
    cout << ">>>>>销毁AIUI代理!\n" << endl;  
    cout << ">>>>>停止所有录音\n" << endl;
}

/**
 * @brief 开始录制降噪音频。
 * @return 总是返回 true。
 */
bool AudioRecorder::startRecord(){

    cout << ">>>>>开始录降噪音频\n" << endl;
    return true;
}

/**
 * @brief 停止录制降噪音频。
 */
void AudioRecorder::stopRecord(){

    cout << ">>>>>停止录降噪音频\n" << endl;
}

/**
 * @brief 开始录制原始音频。
 * @return 总是返回 true。
 */
bool AudioRecorder::startRecordOriginal(){


    cout << ">>>>>开始录音原始音频\n" << endl;
    return true;
}

/**
 * @brief 停止录制原始音频。
 */
void AudioRecorder::stopRecordOriginal(){

    cout << ">>>>>停止录原始音频\n" << endl;
}

/**
 * @brief 设置录音角度。
 * @param angle 录音角度。
 * @return 如果成功启动则返回 true，否则返回 false。
 */
bool AudioRecorder::setRecordAngle(int angle){
    if(!if_success_boot)
    {
        return false;
    }
    //set_mic_angle(angle);
    return true;
}


