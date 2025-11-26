/**
 * @file TestListener.cpp
 * @brief TestListener 类的实现文件。
 * 该文件包含了 TestListener 类的构造函数和析构函数的实现，主要用于管理 TTS (Text-to-Speech) 文件操作的辅助对象。
 */

#include <TestListener.h>

/**
 * @brief TestListener 类的构造函数。
 * 初始化 mTtsFileHelper，创建一个 FileUtil::DataFileHelper 实例，用于处理 TTS 相关的音频文件。
 */
TestListener::TestListener()
{
	mTtsFileHelper = new FileUtil::DataFileHelper("");
}

/**
 * @brief TestListener 类的析构函数。
 * 释放 mTtsFileHelper 占用的内存，确保资源被正确清理。
 */
TestListener::~TestListener()
{
	if (mTtsFileHelper != NULL)
	{
		delete mTtsFileHelper;
		mTtsFileHelper = NULL;
	}
}