#ifndef GLOBAL_AIUI_H_
#define GLOBAL_AIUI_H_

#include <iostream>
#include "aiui/AIUI.h"
#include "FileUtil.h"
#include <AudioPlayer.h>
#include <map>
#include <serial/serial.h>

#define POS_THREAD 1
#define RPY_THREAD 10

/************************初始化参数或固定参数，非必要可不修改**********************************/
IAIUIAgent* globalAgent;
AudioPlayer* globalAudioPlayer;
char* package_path;
char* package_path1;                                                        // 用于存储功能包的绝对路径
serial::Serial _serial;                                                  // 初始化参数,用于串口设置和读写
int angle;                                                               // 初始化参数
char question[128];                                                      // 初始化参数
char answer[128];                                                        // 初始化参数
int sign_conversation_cloud = 0;                                         // 初始化参数,当识别结果来源于在线识别引擎时为1,否则为0
int sign_conversation_local = 0;                                         // 初始化参数,当识别结果来源于离线识别引擎时为1,否则为0
char* TEST_ROOT_DIR =(char*) "";                                         // 初始化参数                     
//配置文件打的路径，里面是客户端设置的参数
char* TEST_AUDIO_PATH =(char*) "/config/AIUI/audio/test.pcm";            // 测试音频的路径
char* LOG_DIR = (char*)"/config/AIUI/log";                               // 识别引擎log保存路径
char* TEST_RECORD_PATH =(char*)"/config/AIUI/audio/record.pcm";          
char* CONFIG_FILE_PATH = (char*)"/tmp/config.txt";                    
char* SOURCE_FILE_PATH = (char*)"/tmp/system.tar";
char* PCM_FILE_PATH = (char*)"/audio/hid_aiui_deno.pcm";                 // 若选择保存音频到本地，则保存到此指定路径,为降噪后的音频
char* ORIPCM_FILE_PATH = (char*)"/audio/hid_aiui_ori.pcm";               // 若选择保存音频到本地，则保存到此指定路径,为原始音频，多通道音频交错分布
char* WAKEUP_RESPONSE_WAV = (char*)"/audio/wakeup.mp3";  
char* NO_INTERNET_RESPONSE_WAV = (char*)"/audio/no_internet.wav";        // 检测没有网络的警告声音
std::map<std::string,std::string> QA_list_;
std::map<std::string,std::string> QA_list_doc;
bool wait_for_awake_word = false;                                        // 等待用户说出唤醒词
bool no_tts = false;                                                     // 是否需要在线合成
bool sign_angle = false;                                                 // 麦克风阵列是否被唤醒

int read_flag = 0;
int wakeupflag = 0;
int run_flag =0;
int i;
int err;
snd_pcm_t *capture_handle;// 一个指向PCM设备的句柄
snd_pcm_hw_params_t *hw_params; //此结构包含有关硬件的信息，可用于指定PCM流的配置
char* device = "2,0";

char   rcv_buf[] = {'0'}; 
int data_len = 8;
int fd = 0;
int serial_error = 0;
/*********************************用户可修改参数*******************************************/
#define DEV_ID "/dev/ttyS3"                                            // 外接设备的串口号,若用语音控制机器人运动,则该串口号为机器人下位机
#define BAUD_RATE 115200  

#define PCM_MSG_LEN 1024
#define ORI_PCM_MSG_LEN 16384

#define AudioFormat SND_PCM_FORMAT_S16_LE  //指定音频的格式,其他常用格式：SND_PCM_FORMAT_U24_LE、SND_PCM_FORMAT_U32_LE
#define AUDIO_CHANNEL_SET   1         //1单声道   2立体声
#define AUDIO_RATE_SET 16000   //音频采样率,常用的采样频率: 44100Hz 、16000HZ、8000HZ



#define RECV_BUF_LEN     12
#define MSG_NORMAL_LEN   4
#define MSG_EXTRA_LEN    8
#define PACKET_LEN_BIT   4
#define SYNC_HEAD        0xa5
#define SYNC_HEAD_SECOND 0x01
   
char  *pcm_name = "hw:XFMDPV0018";                                              // 选择使用的设备型号
//char  *pcm_name = "hw:2,0";                                              // 选择使用的设备型号
int buffer_frames = 512;
char *buffer1;
bool if_print_proc_log = false;                                          // 是否打印log,调试用
bool if_save_record_file = true;                                         // 是否保存音频到本地
std::string USER_CONFIG_PATH = "/config/offline_QA.txt";                 // 离线问答对保存的地址
string appid = "ff44ed3f";                                               // 仅用于在线AIUI，用户可修改，普通用户每天500次
string key = "48b555abbd6a8aecb5344dfaf50aa640";                         // AIUI引擎Appid对应的AppKey
string offline_appid = "appid = ff44ed3f";					 //仅用于离线命令词识别I，用户可修改.
bool offline_mode = true;                                                // 若为true则开启离线识别引擎，否则仅为在线识别引擎
char* GRAMMAR_FILE_PATH = (char*)"/config/call.bnf";                     // 离线引擎的语法文件,该变量的路径可不修改，但对应的bnf文件需要用户根据自己期望的待识别词语进行修改，可参看bnf语法
char* CFG_FILE_PATH = (char*)"/config/AIUI/cfg/aiui.cfg";                // 语音识别配置文件地址,可根据需要修改该cfg文件里的参数
#endif /* GLOBAL_AIUI_H_ */
