/**
 * @file WriteAudioThread.cpp
 * @brief 音频写入线程实现文件。
 * 该文件实现了 WriteAudioThread 类，用于在一个单独的线程中读取音频数据并发送给 AIUI 代理。
 */

#include <WriteAudioThread.h>

/**
 * @brief 线程循环函数。
 * 从音频文件中读取数据，并将其作为 AIUI 消息发送。如果文件读取完毕，根据 mRepeat 标志决定是重绕文件还是停止写入。
 * @return 如果线程需要继续运行则返回 true，否则返回 false。
 */
bool WriteAudioThread::threadLoop()
{
	char audio[1280];
	int len = mFileHelper->read(audio, 1280);

	if (len > 0)
	{
		Buffer* buffer = Buffer::alloc(len);//申请的内存会在sdk内部释放
		memcpy(buffer->data(), audio, len);

		IAIUIMessage * writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE,
			0, 0,  "data_type=audio,sample_rate=16000", buffer);	

		if (NULL != mAgent)
		{
			mAgent->sendMessage(writeMsg);
		}		
		writeMsg->destroy();
		usleep(40 * 1000); // 暂停 40 毫秒，控制音频发送速率
	} else {
		if (mRepeat)
		{
			mFileHelper->rewindReadFile(); // 重绕文件，从头开始读取
		} else {
			IAIUIMessage * stopWrite = IAIUIMessage::create(AIUIConstant::CMD_STOP_WRITE,
				0, 0, "data_type=audio,sample_rate=16000");

			if (NULL != mAgent)
			{
				mAgent->sendMessage(stopWrite);
			}
			stopWrite->destroy();

			mFileHelper->closeReadFile(); // 关闭音频文件
			mRun = false; // 停止线程运行
		}
	}

	return mRun;
}

/**
 * @brief 线程入口函数。
 * 这是 pthread_create 调用的静态函数，它将 paramptr 转换为 WriteAudioThread 实例并调用其 threadLoop 方法。
 * @param paramptr 指向 WriteAudioThread 实例的指针。
 * @return 线程退出状态，通常为 NULL。
 */
void* WriteAudioThread::thread_proc(void * paramptr)
{
	WriteAudioThread * self = (WriteAudioThread *)paramptr;

	while (1) { // 循环调用 threadLoop 直到其返回 false
		if (! self->threadLoop())
			break;
	}
	return NULL;
}

/**
 * @brief WriteAudioThread 类的构造函数。
 * 初始化 AIUI 代理、音频路径、重复播放标志、运行状态和文件辅助对象，并打开音频文件。
 * @param agent AIUI 代理实例，用于发送消息。
 * @param audioPath 音频文件的路径。
 * @param repeat 是否重复播放音频。
 */
WriteAudioThread::WriteAudioThread(IAIUIAgent* agent, const string& audioPath, bool repeat):
mAgent(agent), mAudioPath(audioPath), mRepeat(repeat), mRun(true), mFileHelper(NULL)
,thread_created(false)
{
	mFileHelper = new FileUtil::DataFileHelper(""); // 创建文件辅助对象
	mFileHelper->openReadFile(mAudioPath, false); // 打开音频文件进行读取
}

/**
 * @brief WriteAudioThread 类的析构函数。
 * 释放文件辅助对象 mFileHelper 占用的内存。
 */
WriteAudioThread::~WriteAudioThread()
{
	if (NULL != mFileHelper)
	{
		delete mFileHelper;
		mFileHelper = NULL;
	}
}

/**
 * @brief 停止线程运行。
 * 设置运行标志为 false，并等待线程结束。
 */
void WriteAudioThread::stopRun()
{
    if (thread_created) { // 检查线程是否已创建
        mRun = false; // 设置运行标志为 false，通知线程停止
        void * retarg;
        pthread_join(thread_id, &retarg); // 等待线程结束
        thread_created = false; // 重置线程创建标志
    }
}

/**
 * @brief 启动线程。
 * 创建一个新的 POSIX 线程来执行 thread_proc 函数。
 * @return 如果线程成功创建则返回 true，否则返回 false。
 */
bool WriteAudioThread::run()
{
    if (thread_created == false) { // 检查线程是否未创建
        int rc = pthread_create(&thread_id, NULL, thread_proc, this); // 创建线程
        if (rc != 0) { // 线程创建失败
            exit(-1); // 退出程序
        }
        thread_created = true; // 设置线程创建标志
        return true;
    }

    return false; // 线程已创建，无需再次创建
}