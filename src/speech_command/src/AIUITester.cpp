/**
 * @file AIUITester.cpp
 * @brief AIUI 测试器实现文件。
 * 该文件包含了 AIUI 语音交互的测试逻辑，包括音频播放、唤醒词设置、业务回调处理以及 AIUI 事件监听等。
 */

#include <AIUITester.h>
// #include <msp_cmn.h>
// #include <msp_errors.h>

#define MAX_BUFFER 2097152 /**< @brief 环形缓冲区最大大小。 */

//using namespace VA;

// 声明全局函数，用于唤醒、休眠和清除音频文件
void gWakeup();
void gSleep();
void clearAudioFile(char *fileName);

static RingBuffer buffer_source(MAX_BUFFER); /**< @brief 用于音频数据传输的环形缓冲区。 */
unsigned int rate = AUDIO_RATE_SET; /**< @brief 音频采样率设置。 */

/**
 * @brief 使用 RingBuffer 播放音频。
 * 该函数打开 ALSA PCM 设备，设置参数，然后从 "tts_sample.wav" 文件读取数据并播放。
 */
void play()
{
    snd_pcm_t *pcm_handle;
    // 打开 PCM 设备进行播放
    snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    // 设置 PCM 设备参数：格式、访问模式、声道数、采样率、重采样、周期时间
    snd_pcm_set_params(pcm_handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 2, 44100, 1, 500000);
    FILE *fp = fopen("tts_sample.wav", "rb");
    int buf_size = 1000000;
    char buf[buf_size];
    while (1)
    {
        int size = fread(buf, 1, buf_size, fp);
        if (size == 0)
        {
            break;
        }
        int ret = snd_pcm_writei(pcm_handle, buf, size);
        if (ret == -EPIPE) // 如果发生欠载，则准备 PCM 设备
        {
            snd_pcm_prepare(pcm_handle);
        }
    }
    snd_pcm_drain(pcm_handle); // 等待所有帧播放完毕
    snd_pcm_close(pcm_handle); // 关闭 PCM 设备
}

/**
 * @brief 从环形缓冲区读取数据并播放。
 * 该函数从 `buffer_source` 环形缓冲区读取音频数据，并通过 `globalAudioPlayer` 进行播放。
 * @return void* 线程函数通常返回 void*。
 */
void *read_and_play1()
{
	if (buffer_source.get_length() > 0)
	{
		read_flag = 1; // 设置读取标志位
		char data_to_speech[200000];
		int res1 = buffer_source.RingBuff_Tx(data_to_speech, buffer_source.get_length()); // 从环形缓冲区读取数据
		read_flag = 0; // 清除读取标志位
		if (res1 != -1)
		{
			globalAudioPlayer->Write((unsigned char *)data_to_speech, res1); // 播放音频数据
		}
	}
}


/**
 * @brief 设置一次性唤醒词。
 * 该函数对传入的唤醒词字符串进行处理，移除末尾的 5 个字符和开头的 1 个字符，然后设置唤醒词。
 * @param awake_word 原始唤醒词字符串。
 */
void set_awake_word_once(std::string awake_word)
{
	for (int i = 0; i < 5; i++)
	{
		awake_word.pop_back(); // 移除末尾字符
	}
	awake_word.erase(0, 1); // 移除开头字符
	set_awake_word((char *)awake_word.c_str()); // 设置唤醒词
}

/**
 * @brief 业务处理回调函数。
 * 该函数是 `AudioRecorder` 类的成员函数，用于监听底层业务应答。
 * @param businessMsg 业务消息结构体。
 * @return int 返回处理结果。
 */
int AudioRecorder::business_proc_callback(business_msg_t businessMsg)
{


}

/**
 * @brief 设置语义合成时的发音人参数。
 * 该函数创建一个 AIUI 消息，设置 `audioparams` 中的 `vcn`（发音人）为 "x2_xiaojuan"，然后发送给 AIUI 代理。
 */
void setParams()
{
	char *setParams = (char*)"{\"audioparams\":{\"vcn\":\"x2_xiaojuan\"}}"; // 设置发音人参数的 JSON 字符串
	IAIUIMessage *setMsg = IAIUIMessage::create(AIUIConstant::CMD_SET_PARAMS, 0, 0, setParams, NULL); // 创建设置参数消息
	globalAgent->sendMessage(setMsg); // 发送消息给 AIUI 代理
}


/**
 * @brief AIUI 事件回调接口。
 * SDK 状态、文本、语义结果等都是通过该接口抛出，是核心代码。
 * @param event 接收到的 AIUI 事件。
 */
void TestListener::onEvent(const IAIUIEvent &event) const
{
	if (if_print_proc_log)
	{
		cout << "onEvent" << endl;
	}
	switch (event.getEventType())
	{
	// SDK 状态回调
	case AIUIConstant::EVENT_STATE: // 服务状态事件
	{
		switch (event.getArg1())
		{
		case AIUIConstant::STATE_IDLE:
		{
			cout << ">>>>>EVENT_STATE:"
				 << "IDLE" << endl;
		}
		break;

		case AIUIConstant::STATE_READY:
		{
			cout << ">>>>>EVENT_STATE:"
				 << "READY" << endl;
		}
		break;

		case AIUIConstant::STATE_WORKING:
		{
			cout << ">>>>>EVENT_STATE:"
				 << "WORKING" << endl;
		}
		break;
		}
	}
	break;

	/* 唤醒事件回调
    arg1字段取值(自1051版本开始支持)：
    0 => 内部语音唤醒
    1 => 外部手动唤醒（外部发送CMD_WAKEUP）。
    info字段为唤醒结果JSON字符串。*/
	case AIUIConstant::EVENT_WAKEUP:
	{
		//cout << "EVENT_WAKEUP:" << event.getInfo() << endl;
		cout << ">>>>>麦克风已唤醒，可进行对话" << endl;
	}
	break;

	// 休眠事件回调：当出现交互超时，服务会先抛出准备休眠事件，用户可在接收到该事件后的10s内继续交互。若10s内存在有效交互，则重新开始交互计时；若10s内不存在有效交互，则10s后进入休眠状态。
	case AIUIConstant::EVENT_SLEEP:
	{
		cout << ">>>>>麦克风准备进入休眠模式，将休眠" << endl;
		//cout <<"VENT_SLEEP:arg1=" << event.getArg1() << endl;
	}
	break;

	// VAD 事件回调：当检测到输入音频的前端点后，会抛出该事件，用arg1标识前后端点或者音量信息:0(前端点)、1(音量)、2(后端点)、3（前端点超时）当arg1取值为1时，arg2为音量大小。
	case AIUIConstant::EVENT_VAD:
	{
		switch (event.getArg1())
		{
		case AIUIConstant::VAD_BOS:
		{
			cout << ">>>>>检测到对话开始" << endl;
		}
		break;

		case AIUIConstant::VAD_EOS:
		{
			cout << ">>>>>检测到本轮对话结束" << endl;
		}
		break;

		case AIUIConstant::VAD_VOL:
		{
			// event.getData
			// cout << "EVENT_VAD:" << "VOL " << endl;
			// cout << "EVENT_VAD:" << "EVENT_INFO:" << event.getInfo() << endl;
			// cout << "EVENT_VAD:" << "GET_ARG2  :" << event.getArg2() << endl;
			// cout << "EVENT_VAD:" << "GET_ARG2  :" << event.getArg2() << endl;
		}
		break;
		}
		// cout << "EVENT_VAD:"<< "EVENT_RESULT:" << event.getData()->getString("result", "") << endl;
	}
	break;
	// case AIUIConstant::EVENT_TTS:{
	// 	cout << "AIUIConstant::EVENT_TTS" << endl;

	// }
	// 最重要的结果事件回调
	case AIUIConstant::EVENT_RESULT: // data字段携带结果数据，info字段为描述数据的JSON字符串。
	{
		Json::Value bizParamJson;
		Json::Reader reader;

		if (!reader.parse(event.getInfo(), bizParamJson, false))
		{
			cout << "解析错误!" << endl
				 << event.getInfo() << endl;
			break;
		}
		Json::Value data = (bizParamJson["data"])[0];
		Json::Value params = data["params"];
		Json::Value content = (data["content"])[0];
		string sub = params["sub"].asString();
		//sub字段表示结果的类别，如iat听写，nlp语义结果
#if if_print_proc_log
		//cout << ">>>>>当前事件状态:" << data << endl;
#endif
		if (sub == "asr")
		{
			Json::Value empty;
			Json::Value contentId = content.get("cnt_id", empty);
			if (contentId.empty())
			{
				cout << "Content Id is empty" << endl;
				break;
			}
			string cnt_id = contentId.asString();
			int dataLen = 0;
			const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
			cout << "buffer:" << buffer << endl;
			string resultStr;

			if (NULL != buffer)
			{
				resultStr = string(buffer, dataLen);
//#if if_print_proc_log
				cout << "resultstr:" << resultStr << endl;
//#endif	
				Json::Reader resultReader;
				Json::Value resultJson;
				resultReader.parse(resultStr, resultJson);
				Json::Value result_category = resultJson["intent"]["category"];
				std::string res_question_str = "";
				if (resultStr.size() > 20)
				{
					int number = resultJson["intent"]["ws"].size();
					for (int i = 0; i < number; i++)
					{
						Json::Value current_word_ = resultJson["intent"]["ws"][i]["cw"][0]["w"];
						std::string current_word = string(current_word_.toStyledString());
						current_word.pop_back();
						current_word.pop_back();
						res_question_str += current_word.substr(1, current_word.size() - 1);
					}

					cout << ">>>>>离线问题是:" << res_question_str << endl;
#if if_print_proc_log
#endif
					strcpy(question, res_question_str.c_str());
					strcpy(answer, "默认回答,收到");
					sign_conversation_local = 1;

					// int pos_str1 = resultStr.rfind("slot") + 7; //>>>>>slot位置
					// //string tmp(resultStr,pos_str1);
					// int size = resultStr.rfind('"') - pos_str1;			 //子字符串的长度
					// string outresult = resultStr.substr(pos_str1, size); //输出结果
					// cout << ">>>>>输出结果:" << outresult << endl;
					// //system(play_audio_path.c_str());
					// //if_recognise = true;
					// //cout << "########number######:" << command << endl;
					// //Trans_Ros(command);
				}
			}
		}
		else if (sub == "nlp")
		{
			Json::Value empty;
			Json::Value contentId = content.get("cnt_id", empty);

			if (contentId.empty())
			{
				cout << "Content Id is empty" << endl;
				break;
			}

			string cnt_id = contentId.asString();
			int dataLen = 0;
			const char *buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
			string resultStr;
			//---------------------

			//+++++++++++++++++++++
			if (NULL != buffer)
			{
				resultStr = string(buffer, dataLen);
				Json::Reader resultReader;
				Json::Value resultJson;
				resultReader.parse(resultStr, resultJson);
				Json::Value result_category = resultJson["intent"]["category"];
				std::string res_str = result_category.toStyledString();
				std::string skill_id = "\"OS1763379980.sleep1\"\n";

				if (res_str == skill_id)
				{
					gSleep();
					//gTTS("好的,那我先走了!");
				}

				if (resultStr.size() > 20)
				{
					Json::Value result_question = resultJson["intent"]["text"];
					std::string res_question_str = result_question.toStyledString();

					Json::Value result_answer = resultJson["intent"]["answer"]["text"];
					std::string res_answer_str = result_answer.toStyledString();
#if if_print_proc_log
					cout << ">>>>>在线问题是:" << res_question_str << endl;
					cout << ">>>>>在线答案是:" << res_answer_str << endl;
#endif

					/****************************/
					strcpy(question, res_question_str.c_str());
					strcpy(answer, res_answer_str.c_str());
					sign_conversation_cloud = 1;

					std::string face_regist("我叫");
					if (res_question_str.find(face_regist) != string::npos)
					{
						std::cout << "no_tts" << no_tts << std::endl;
						no_tts = true;
					}
					else
					{
						no_tts = false;
					}

					/****************************/
					if (wait_for_awake_word)
					{
						set_awake_word_once(res_question_str);
						wait_for_awake_word = false;
						no_tts = true;
						//gTTS("设置成功,你可以叫我" + res_question_str);
					}
					std::string result_hope("那给我起个4到6个字的新名字吧");
					if (res_answer_str.find(result_hope) != string::npos)
					{
						wait_for_awake_word = true;
					}
					//else
					//{
					//no_tts = false;
					//}
				}
			}
			else
			{
				cout << "buffer is NULL" << endl;
			}
		}
		else if (sub == "tts" && !no_tts)
		{
			static int count_tts = 0;
			//cout << event.getInfo() << endl;

			string cnt_id = content["cnt_id"].asString();

			int dataLen = 0;
			const char *data = event.getData()->getBinary(cnt_id.c_str(), &dataLen);

			int dts = content["dts"].asInt();

			string errorinfo = content["error"].asString();

			if (2 == dts && errorinfo == "AIUI DATA NULL")
			{
				//满足这两个条件的是空结果,不处理,直接丢弃
			}
			else if (3 == dts)
			{
				//cout << "tts result receive dts = 3 result" << endl;
				mTtsFileHelper->createWriteFile("tts", ".pcm", false);
				mTtsFileHelper->write((const char *)data, dataLen, 0, dataLen);
				mTtsFileHelper->closeFile();
			}
			else
			{
				if (0 == dts)
				{
					sleep(0.0000001);
					globalAudioPlayer->Clear_Write();
					mTtsFileHelper->createWriteFile("tts", ".pcm", false);
				}
				if (read_flag == 0)
				{

					//each_time_read = dataLen;
#if if_print_proc_log
					cout << "[**import location] buffer's write location:" << buffer_source.get_tail() << endl;
#endif
					int res = buffer_source.RingBuff_Rx((char *)data, dataLen);
#if if_print_proc_log
					cout << "[**import log**] write buffer's curret length:" << buffer_source.get_length() << endl;
#endif
					//read_and_play1();
					mTtsFileHelper->write((const char *)data, dataLen, 0, dataLen);

					if (2 == dts)
					{
						mTtsFileHelper->closeWriteFile();
					}
				}
			}
		}
	}
	break;

	/*上传资源数据的返回结果某条CMD命令对应的返回事件,对于除CMD_GET_STATE外的有返回的命令，都会返回该事件，
    用arg1标识对应的CMD命令，arg2为返回值，0表示成功，
    info字段为描述信息。*/
	case AIUIConstant::EVENT_CMD_RETURN:
	{
		//cout << "onEvent --> EVENT_CMD_RETURN: arg1 is " << event.getArg1() << endl;
		if (AIUIConstant::CMD_SYNC == event.getArg1())
		{
			int retcode = event.getArg2();
			int dtype = event.getData()->getInt("sync_dtype", -1);

			//cout << "onEvent --> EVENT_CMD_RETURN: dtype is " << dtype << endl;

			switch (dtype)
			{
			case AIUIConstant::SYNC_DATA_STATUS:
				break;

			case AIUIConstant::SYNC_DATA_ACCOUNT:
				break;

			case AIUIConstant::SYNC_DATA_SCHEMA:
			{
				string sid = event.getData()->getString("sid", "");
				string tag = event.getData()->getString("tag", "");

				string mSyncSid = sid;

				if (AIUIConstant::SUCCESS == retcode)
				{
					cout << "sync schema success." << endl;
				}
				else
				{
					cout << "sync schema error=" << retcode << endl;
				}

				cout << "sid=" << sid << endl;
				cout << "tag=" << tag << endl;
			}
			break;

			case AIUIConstant::SYNC_DATA_SPEAKABLE:
				break;

			case AIUIConstant::SYNC_DATA_QUERY: //查询结果
			{
				if (AIUIConstant::SUCCESS == retcode)
				{
					cout << "sync status success" << endl;
				}
				else
				{
					cout << "sync status error=" << retcode << endl;
				}
			}
			break;
			}
		}
		else if (AIUIConstant::CMD_QUERY_SYNC_STATUS == event.getArg1())
		{
			int syncType = event.getData()->getInt("sync_dtype", -1);
			if (AIUIConstant::SYNC_DATA_QUERY == syncType)
			{
				string result = event.getData()->getString("result", "");
				cout << "result:" << result << endl;

				if (0 == event.getArg2())
				{
					cout << "sync status:success." << endl;
				}
				else
				{
					cout << "sync status error:" << event.getArg2() << endl;
				}
			}
		}
		else if (AIUIConstant::CMD_BUILD_GRAMMAR == event.getArg1())
		{
			if (event.getArg2() == 0)
			{
				cout << "build grammar success." << endl;
			}
			else
			{
				cout << "build grammar error, errcode = " << event.getArg2() << endl;
				cout << "error reasion is " << event.getInfo() << endl;
			}
		}
		else if (AIUIConstant::CMD_UPDATE_LOCAL_LEXICON == event.getArg1())
		{
			if (event.getArg2() == 0)
			{
				cout << "update lexicon success" << endl;
			}
			else
			{
				cout << "update lexicon error, errcode = " << event.getArg2() << endl;
				cout << "error reasion is " << event.getInfo() << endl;
			}
		}
	}
	break;

	case AIUIConstant::EVENT_ERROR: //arg1字段为错误码，info字段为错误描述信息。
	{
		cout << "EVENT_ERROR:" << dec << event.getArg1() << endl;
		cout << " ERROR info is " << event.getInfo() << endl;
		//string networkerror="10120";
		if (wakeupflag == 1 && event.getArg1() == 10120)
		{
			cout << " 网络连接失败，将尝试使用离线识别！ " << endl;
			std::string wav_path = NO_INTERNET_RESPONSE_WAV;
			std::string command = "play " + wav_path;
			system(command.c_str());
			wakeupflag = 0;
		}
	}
	break;
	}
}

/**
 * @brief 创建AIUI代理
 * 
 */
void AIUITester::createAgent()
{
	Json::Value paramJson;
	Json::Value appidJson;

	appidJson["appid"] = appid;
	appidJson["key"] = key;
	string fileParam = FileUtil::readFileAsString(CFG_FILE_PATH);
	Json::Reader reader;
	if (reader.parse(fileParam, paramJson, false))
	{
		paramJson["login"] = appidJson;

		//for ivw support
		string wakeup_mode = paramJson["speech"]["wakeup_mode"].asString();
		string engine_type = paramJson["speech"]["intent_engine_type"].asString();
		// 如果在aiui.cfg中设置了交互模式为离线(local)或混合(mixed)交互模式，需要初始化msc并构建语法
		if (engine_type != "cloud")
		{
			cout << "engine type is not cloud" << endl;
			string lgiparams = offline_appid;
			int ret = MSPLogin(NULL, NULL, lgiparams.c_str());
			if (0 != ret)
			{
				printf(">>>>>MSP登录失败：%d\n", ret);
			}
		}
		//如果在aiui.cfg中设置了唤醒模式为ivw唤醒，那么需要对设置的唤醒资源路径作处理，并且设置唤醒的libmsc.so的路径为当前路径
		if (wakeup_mode == "ivw")
		{

			//readme中有说明，使用libmsc.so唤醒库，需要调用MSPLogin()先登录
			// string lgiparams = "appid=5d3fbe80,engine_start=ivw";
			// string lgiparams = "appid=5d3fbe80";
			// MSPLogin(NULL, NULL, lgiparams.c_str());
			string ivw_res_path = paramJson["ivw"]["res_path"].asString();
			if (!ivw_res_path.empty())
			{
				ivw_res_path = "fo|" + ivw_res_path;
				paramJson["ivw"]["res_path"] = ivw_res_path;
			}

			string ivw_lib_path = "libmsc.so";

			paramJson["ivw"]["msc_lib_path"] = ivw_lib_path;
		}
		//end
		Json::FastWriter writer;
		string paramStr = writer.write(paramJson);
		agent = IAIUIAgent::createAgent(paramStr.c_str(), &listener);
		globalAgent = agent;
		globalAudioPlayer = new AudioPlayer();
		setParams();
		//gTTS("这是一段测试音频，这是一段测试音频，科大讯飞智能驾驶套件测试音频");
		//play();
	}
	else
	{
		cout << ">>>>>aiui.cfg 读取错误!" << endl;
	}
}

/*
	外部唤醒接口，通过发送CMD_WAKEUP命令对SDK进行外部唤醒，发送该命令后，SDK会进入working状态，用户就可以与SDK进行交互。
*/
void AIUITester::wakeup()
{
	if (NULL != agent)
	{
		IAIUIMessage *wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_WAKEUP);
		agent->sendMessage(wakeupMsg);
		wakeupMsg->destroy();
	}
}

void gWakeup()
{
	if (NULL != globalAgent)
	{
		IAIUIMessage *wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_WAKEUP);
		globalAgent->sendMessage(wakeupMsg);
		wakeupMsg->destroy();
	}
}
void gSleep()
{
	// gTTS("我去休息了");
	if (NULL != globalAgent)
	{
		IAIUIMessage *sleepMsg = IAIUIMessage::create(AIUIConstant::CMD_RESET_WAKEUP);
		globalAgent->sendMessage(sleepMsg);
		sleepMsg->destroy();
	}
	globalAudioPlayer->Clear_Write();
}

void gTTS(string text)
{
	if (NULL != globalAgent)
	{
		Buffer *textData = Buffer::alloc(text.length());
		text.copy((char *)textData->data(), text.length());
		string paramStr = "vcn=x2_xiaojuan"; //xiaoyan x2_pengfei x2_qige x2_yifei
		paramStr += ",speed=40";
		paramStr += ",pitch=50";
		paramStr += ",volume=80";
		paramStr += ",aue=speex-wb;7";

		IAIUIMessage *ttsMsg = IAIUIMessage::create(AIUIConstant::CMD_TTS,AIUIConstant::START, 0, paramStr.c_str(), textData);

		globalAgent->sendMessage(ttsMsg);

		ttsMsg->destroy();
	}
}

//停止AIUI服务，此接口是与stop()对应，调用stop()之后必须调用此接口才能继续与SDK交互，如果你没有调用过stop(),就不需要调用该接口
void AIUITester::start()
{
	if (NULL != agent)
	{
		IAIUIMessage *startMsg = IAIUIMessage::create(AIUIConstant::CMD_START);
		agent->sendMessage(startMsg);
		startMsg->destroy();
	}
}

//停止AIUI服务
void AIUITester::stop()
{
	if (NULL != agent)
	{
		IAIUIMessage *stopMsg = IAIUIMessage::create(AIUIConstant::CMD_STOP);
		agent->sendMessage(stopMsg);
		stopMsg->destroy();
	}
}

void AIUITester::reset()
{
	if (NULL != agent)
	{
		IAIUIMessage *resetMsg = IAIUIMessage::create(AIUIConstant::CMD_RESET);
		agent->sendMessage(resetMsg);
		resetMsg->destroy();
	}
}

void AIUITester::destory()
{
	if (audioRecorder != NULL)
	{
		audioRecorder->~AudioRecorder();
		audioRecorder = NULL;
	}

	if (NULL != agent)
	{
		agent->destroy();
		// globalAgent ->destroy();
		agent = NULL;
		// globalAgent= NULL;
	}
	// sleep(10);
	//cout << " agent::destory()" << endl;
}

void AIUITester::recorder_creat()
{
	if (agent == NULL)
	{
		cout << ">>>>>未创建麦克风录音代理\n"<< endl;
		return;
	}
	if (audioRecorder == NULL)
	{
		audioRecorder = new AudioRecorder(TEST_RECORD_PATH);
	}
}

void AIUITester::recorder_start()
{
	bool first_log = false;
	while (!AudioRecorder::if_success_boot)
	{
		if (AudioRecorder::if_awake)
		{
			break;
		}
		if (!first_log)
		{
			cout << ">>>>>请使用唤醒词唤醒\n"
				 << endl;
			first_log = true;
		}
		sleep(1);
	}
	//audioRecorder->startRecord();
	if (if_save_record_file)
	{
		//audioRecorder->startRecordOriginal();
	}
}

void AIUITester::recorder_stop()
{
	// cout << "recorder_stop" << endl;
	audioRecorder->stopRecord();
}

void clearAudioFile(char *fileName)
{
	FILE *pFile = fopen(fileName, "w+");
	unsigned char *buf;
	fwrite(buf, sizeof(char), 0, pFile);
	fclose(pFile);
	pFile = NULL;
}

AIUITester::AIUITester() : agent(NULL), audioRecorder(NULL), audioPlayer(NULL)
{
	// if (audioPlayer == NULL){
	// 	audioPlayer = new AudioPlayer();
	// 	// audioRecoder->run();
	// }
}

AIUITester::~AIUITester()
{
	if (agent != NULL)
	{
		agent->destroy();
		cout << ">>>>>销毁AIUI代理!\n" << endl;  
		cout << ">>>>>停止所有录音\n" << endl;

		agent = NULL;
	}
}

void AIUITester::buildGrammar()
{
	if (NULL != agent)
	{
		string grammarContent = FileUtil::readFileAsString(GRAMMAR_FILE_PATH);
		IAIUIMessage *grammarMsg = IAIUIMessage::create(AIUIConstant::CMD_BUILD_GRAMMAR, 0, 0, grammarContent.c_str());

		agent->sendMessage(grammarMsg);

		grammarMsg->destroy();
	}
}

void AIUITester::updateLocalLexicon()
{
	if (NULL != agent)
	{
		Json::Value paramJson;

		paramJson["name"] = "contact";
		paramJson["content"] = "张山\n李思\n刘得花\n";

		Json::FastWriter writer;
		string paramStr = writer.write(paramJson);

		IAIUIMessage *updateMsg = IAIUIMessage::create(AIUIConstant::CMD_UPDATE_LOCAL_LEXICON, 0, 0, paramStr.c_str(), NULL);

		agent->sendMessage(updateMsg);

		updateMsg->destroy();
	}
}

//接收用户输入命令，调用不同的测试接口
void AIUITester::readCmd()
{
	string cmd;
	while (1)
	{
		cin >> cmd;
		if (cmd == "q")
		{
			destory();
			break;
		}
	}
}


static string MakeMsgPacket(unsigned short sid, MsgType type, const string &content)
{
    const unsigned short size = content.size();

    string data;

    data += (char)0xA5;                 /* head     */
    data += (char)0x01;                 /* uid      */
    data += (char)type;                 /* type     */
    data += (char)(size & 0xff);        /* len_low  */
    data += (char)((size >> 8) & 0xff); /* len_high */
    data += (char)(sid & 0xff);         /* sid_low  */
    data += (char)((sid >> 8) & 0xff);  /* sid_high */

#if 0
    if (content.length() > 1024) {
        QByteArray compressedData;
        QCompressor::gzipCompress(content, compressedData);
        data += compressedData;
    } else
#endif
    {
        data += content;
    }

    int sum = std::accumulate(data.cbegin(), data.cend(), 0);

    data += (char)((~sum + 1) & 0xff);

    return data;
}
static bool UnPackMsgPacket(const string &content, MsgPacket &data)
{
    if (content.size() < 7 || ((unsigned char)content.at(0) != 0xA5)) 
    return false;

    data.uid = content[1] & 0xff;
    data.type = content[2] & 0xff;

    data.size = ((content[3] & 0xff) | (content[4] << 8 & 0xff00));
    data.sid = ((content[5] & 0xff) | (content[6] << 8 & 0xff00));

    switch ((MsgType)data.type) 
    {
        case MsgType::AIUI_MSG: 
	{
            string info = content.substr(7, data.size);
#if 0
            if (QCompressor::isCompressed(info)) {
                QCompressor::gzipDecompress(info, data.bytes);
            } else
#endif
            {
                data.bytes = info;
            }

            return true;
        } 
	break;
        default:
            break;
    }

    return false;
}
static string MakeComfirm(short sid)
{
    string data;

    data += (char)0xA5;
    data += (char)0x00;
    data += (char)0x00;
    data += (char)0x00;

    return MakeMsgPacket(sid, MsgType::CONFIRM, data);
}
void process_recv(const unsigned char *buf, int len)
{
    if (buf[2] == 0xff) 
    {
        if (((buf[5] & 0xff) | (buf[6] << 8 & 0xff00)) == 0) 
   	{    //
        }
        return;
    }

    int sum = std::accumulate(buf, buf + len - 1, 0);

    if (((~sum + 1) & 0xff) != buf[len - 1]) 
    {
        cout<<"recv data not crc, drop\n"<<endl;
        return;
    }

    string data = MakeComfirm(((buf[5] & 0xff) | (buf[6] << 8 & 0xff00)));

    //发送确认消息
    _serial.write(data);

    MsgPacket pkg;

    if (UnPackMsgPacket(string((char *)buf, len), pkg)) 
    {
        if ((MsgType)pkg.type == MsgType::AIUI_MSG) 
	{
            Json::Value WakeupJson;
            Json::Reader reader;

            if (!reader.parse(pkg.bytes, WakeupJson, false)) 
	    {
                return;
            }

            Json::Value content = WakeupJson["content"];

            if (content.isMember("eventType")) 
	    {
                if (content["eventType"].asInt() == 4) 
		{
            	string info = content["info"].asString();
				string result = content["result"].asString();
		    //std::string wav_path = WAKEUP_RESPONSE_WAV;
		    //std::string command = "play "+wav_path;
		    //system(command.c_str());
		    //gWakeup();
		    	cout<<"info: "<<info<<endl;
				cout<<"content: "<<content<<endl;
                Json::Value::Members mem = content.getMemberNames();
                Json::Value::Members::iterator it;				
				Json::Reader reader;
				Json::Value root;
				if(reader.parse(info,root))
				{
					angle = root["ivw"]["angle"].asFloat();
					cout << "awake_angle: "<< angle<<endl;
					if(result.length() > 10) {
						if(testCallback != NULL ) {
							testCallback();
						}
					}
				}
				std::string wav_path = WAKEUP_RESPONSE_WAV;
				std::string command = "play "+wav_path;
				system(command.c_str());
				gWakeup();				
                }
            }
        }
    }
}
void uart_rec(const unsigned char *msg, unsigned int msglen)
{
    static int recv_index = 0;
    unsigned char recv_buf[RECV_BUF_LEN];
    static unsigned int big_buf_len = 0;
    static unsigned int big_buf_index = 0;
    static unsigned char *big_buf = NULL;

    //过滤不以A5 01开头的无效数据
    if (big_buf == NULL && recv_index + msglen >= 2) 
    {
        if (recv_index == 0) 
        {
            if (((unsigned char *)msg)[0] != SYNC_HEAD ||((unsigned char *)msg)[1] != SYNC_HEAD_SECOND) 
	    {
                cout<< "recv data not SYNC HEAD, drop"<<endl;
                return;
            }
        }
	else if (recv_index == 1)
	{
            if (recv_buf[0] != SYNC_HEAD || ((unsigned char *)msg)[0] != SYNC_HEAD_SECOND) 
	    {
                cout<<"recv data not SYNC HEAD, drop\n"<<endl;
                recv_index = 0;
                return;
            }
        }
    }

    //不断接收串口字节，构造完成消息
    int copy_len;
    if (big_buf != NULL) 
    {
        copy_len = big_buf_len - big_buf_index < msglen ? big_buf_len - big_buf_index : msglen;
        memcpy(big_buf + big_buf_index, msg, copy_len);
        big_buf_index += copy_len;
        if (big_buf_index < big_buf_len) 
	   return;
    }
    else 
    {
        copy_len = RECV_BUF_LEN - recv_index < msglen ? RECV_BUF_LEN - recv_index : msglen;
        memcpy(recv_buf + recv_index, msg, copy_len);
        if ((recv_index + copy_len) > PACKET_LEN_BIT) 
	{
            unsigned int content_len = recv_buf[PACKET_LEN_BIT] << 8 | recv_buf[PACKET_LEN_BIT - 1];
            if (content_len != MSG_NORMAL_LEN) 
	    {
                big_buf_index = 0;
                big_buf_len = content_len + MSG_EXTRA_LEN;
                big_buf = (unsigned char *)malloc(big_buf_len);
                memset(big_buf, '\0', big_buf_len);
                memcpy(big_buf, recv_buf, recv_index);
                big_buf_index += recv_index;
                recv_index = 0;
                return uart_rec(msg, msglen);
            }
        }
        recv_index += copy_len;
        if (recv_index < RECV_BUF_LEN) 
	   return;
    }

    //已接收完一条完整消息
    if (big_buf != NULL) {
        process_recv(big_buf, big_buf_len);
	//cout<<"success"<<big_buf<<endl;    //接受消息处理
        big_buf_len = 0;
        big_buf_index = 0;
        free(big_buf);
        big_buf = NULL;

    } else {
        process_recv(recv_buf, RECV_BUF_LEN);    //接受消息处理
        recv_index = 0;
    }

    //读取的数据中包含下一条消息的开头部分
    if (copy_len < msglen) {
       cout<< "multi msg in one stream left %d byte\n"<<(msglen - copy_len)<<endl;
        uart_rec(msg + copy_len, msglen - copy_len);
    }
}

//
void exit_sighandler(int sig)
{
	run_flag =1;
}

void AIUITester::bind(TEST_CALLBACK callback)
{
	testCallback = callback;
}

void AIUITester::test()
{
	cout << ">>>>>创建AIUI代理Agent\n"<< endl;
	createAgent();

	cout << ">>>>>正在准备开启麦克风" << endl;
	cout << ">>>>>开启录音" << endl;
        
	if (offline_mode)
	{
		cout << "buildGrammar" << endl;
		buildGrammar();
		//cout << "updateLocalLexicon" << endl;
		//updateLocalLexicon();
	}
	/*注册信号捕获退出接口*/
	signal(2,exit_sighandler);
 
	/*PCM的采样格式在pcm.h文件里有定义*/
	snd_pcm_format_t format=AudioFormat; // 采样位数：16bit、LE格式
	if ((err = snd_pcm_open (&capture_handle, pcm_name ,SND_PCM_STREAM_CAPTURE,0))<0) 
	{
		printf("无法打开音频设备: %s (%s)\n", device,snd_strerror (err));
		exit(1);
	}
	printf("音频接口打开成功.\n");
 
	/*创建一个保存PCM数据的文件*/
	printf("用于录制的音频文件已打开.\n");
 
	/*分配硬件参数结构对象，并判断是否分配成功*/
	if((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) 
	{
		printf("无法分配硬件参数结构 (%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("硬件参数结构已分配成功.\n");
  
	/*按照默认设置对硬件对象进行设置，并判断是否设置成功*/
	if((err=snd_pcm_hw_params_any(capture_handle,hw_params)) < 0) 
	{
		printf("无法初始化硬件参数结构 (%s)\n", snd_strerror(err));
		exit(1);
	}
	printf("硬件参数结构初始化成功.\n");
 
 	/*
    	设置数据为交叉模式，并判断是否设置成功
    	interleaved/non interleaved:交叉/非交叉模式。
    	表示在多声道数据传输的过程中是采样交叉的模式还是非交叉的模式。
    	对多声道数据，如果采样交叉模式，使用一块buffer即可，其中各声道的数据交叉传输；
	如果使用非交叉模式，需要为各声道分别分配一个buffer，各声道数据分别传输。
	*/
	if((err = snd_pcm_hw_params_set_access (capture_handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) 
	{
		printf("无法设置访问类型(%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("访问类型设置成功.\n");
 
	/*设置数据编码格式，并判断是否设置成功*/
	if ((err=snd_pcm_hw_params_set_format(capture_handle, hw_params,format)) < 0) 
	{
		printf("无法设置格式 (%s)\n",snd_strerror(err));
		exit(1);
	}
	fprintf(stdout, "PCM数据格式设置成功.\n");
 
	/*设置采样频率，并判断是否设置成功*/
	if((err=snd_pcm_hw_params_set_rate_near(capture_handle,hw_params,&rate,0))<0) 
	{
		printf("无法设置采样率(%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("采样率设置成功\n");
 
	/*设置声道，并判断是否设置成功*/
	if((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params,AUDIO_CHANNEL_SET)) < 0) 
	{
		printf("无法设置声道数(%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("声道数设置成功.\n");
 
	/*将配置写入驱动程序中，并判断是否配置成功*/
	if ((err=snd_pcm_hw_params (capture_handle,hw_params))<0) 
	{
		printf("无法向驱动程序设置参数(%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("参数设置成功.\n");
 
	/*使采集卡处于空闲状态*/
	snd_pcm_hw_params_free(hw_params);
 
	/*准备音频接口,并判断是否准备好*/
	if((err=snd_pcm_prepare(capture_handle))<0) 
	{
		printf("无法使用音频接口 (%s)\n",snd_strerror(err));
		exit(1);
	}
	printf("音频接口准备好.\n");
 
 	/*配置一个数据缓冲区用来缓冲数据*/
 	int frame_byte=snd_pcm_format_width(format)/8;
	buffer1 = (char*)malloc(buffer_frames*frame_byte*AUDIO_CHANNEL_SET);//申请的内存会在sdk内部释放
	printf("缓冲区分配成功.\n");
  
	/*开始采集音频pcm数据*/
	printf("开始采集数据...\n");
	cout << ">>>>>请使用唤醒词唤醒\n"<< endl;
	while(1) 
	{
		static unsigned char  buff[1024];
		//unsigned char buffer1[];
		memset(buff, '\0', 1024);

		int recLen = 0;
		while (( recLen = _serial.read(buff,_serial.available() ))  > 12) 
		{
			if (recLen > 12)
			{
				//cout<<"ceshiceshi"<<buff<<endl;
				//buffer1 += buff;
            			uart_rec((const unsigned char *)buff, recLen);
       		 	}


		}
		/*从声卡设备读取一帧音频数据:2048字节*/
		if((err=snd_pcm_readi(capture_handle,buffer1,buffer_frames))!=buffer_frames) 
		{
			printf("从音频接口读取失败(%s)\n",snd_strerror(err));
			exit(1);
		}
		//
		Buffer *buffer = Buffer::alloc(buffer_frames*frame_byte*AUDIO_CHANNEL_SET); 
		memcpy(buffer->data(),buffer1,buffer_frames*frame_byte*AUDIO_CHANNEL_SET);
		IAIUIMessage *writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE, 0, 0, "data_type=audio,sample_rate=16000",buffer);  
		if (NULL != globalAgent)
		{
			globalAgent->sendMessage(writeMsg);
		}
		else
		{
			cout << ">>>>>globalAgent未创建\n" << endl;
		}
		writeMsg->destroy();

		if(run_flag)
		{
			printf("停止采集.\n");
			break;
		}

	}
	free(buffer1);
 	_serial.close();
	AIUITester::stop();
	AIUITester::destory();
 	/*关闭音频采集卡硬件*/
	snd_pcm_close(capture_handle);
	AIUISetting::setAIUIDir(TEST_ROOT_DIR);
	AIUISetting::initLogger(LOG_DIR);
}
