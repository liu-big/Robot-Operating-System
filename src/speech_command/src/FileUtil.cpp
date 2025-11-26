/**
 * @file FileUtil.cpp
 * @brief 文件操作工具类实现文件。
 * 该文件包含了文件和目录的创建、删除、判断存在性等实用工具函数。
 */

#include "FileUtil.h"
#include <stdio.h>
#include <string.h>
#include <sstream>

#include "aiui/AIUIType.h"

#if defined(_MSC_VER)
#include <windows.h>
#include <time.h>
#else
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <sys/time.h>
#endif

using namespace std;

/**
 * @brief PCM 文件后缀名。
 */
const string FileUtil::SURFFIX_PCM = ".pcm";

/**
 * @brief TXT 文件后缀名。
 */
const string FileUtil::SURFFIX_TXT = ".txt";

/**
 * @brief CFG 文件后缀名。
 */
const string FileUtil::SURFFIX_CFG = ".cfg";

/**
 * @brief LOG 文件后缀名。
 */
const string FileUtil::SURFFIX_LOG = ".log";

#if defined(_MSC_VER) // Windows 平台特定实现

/**
 * @brief 判断文件或目录是否存在。
 * @param path 文件或目录的路径。
 * @return 如果存在返回 true，否则返回 false。
 */
bool FileUtil::exist(const string &path)
{
	WIN32_FIND_DATA result;
	HANDLE hFile;
	hFile = FindFirstFile(path.c_str(), &result);
	if (hFile == INVALID_HANDLE_VALUE)
		return false;

	FindClose(hFile);
	return true;
}

/**
 * @brief 判断指定路径是否为目录。
 * @param path 路径。
 * @return 如果是目录返回 true，否则返回 false。
 */
bool FileUtil::isDirectory(const string &path)
{
	WIN32_FIND_DATA result;
	HANDLE hFile;
	hFile = FindFirstFile(path.c_str(), &result);
	if (hFile == INVALID_HANDLE_VALUE)
		return false;
	FindClose(hFile);
	if (result.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		return true;	
	return false;
}

/**
 * @brief 如果目录不存在则创建目录。
 * @param path 要创建的目录路径。
 * @return 如果目录已存在或创建成功返回 true，否则返回 false。
 */
bool FileUtil::makeDirIfNoExist(const string &path)
{
	BOOL fret = CreateDirectory(path.c_str(), NULL);
	if (fret == FALSE && ERROR_ALREADY_EXISTS != GetLastError())
		return false;
	return true;
}

/**
 * @brief 删除文件或目录。
 * 如果是目录，则递归删除其内容。
 * @param filePath 要删除的文件或目录的路径。
 * @return 如果删除成功返回 true，否则返回 false。
 */
bool FileUtil::delFile(const string &filePath)
{
	WIN32_FIND_DATA findData;
	HANDLE hFinder;
	string findPath = filePath;
	if (findPath.back() == '\\') {
		findPath.erase(findPath.end()-1);
	}

	hFinder = FindFirstFile(findPath.c_str(), &findData);
	if (hFinder == INVALID_HANDLE_VALUE)
		return false;

	if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
		if (strcmp(findData.cFileName, ".") == 0 || strcmp(findData.cFileName, "..") == 0) {
			FindClose(hFinder);
			return false;
		}

		string newpath = findPath+"\\*";
		WIN32_FIND_DATA fda;
		BOOL fret;

		HANDLE hf = FindFirstFile(newpath.c_str(), &fda);
		if (hf == INVALID_HANDLE_VALUE) {
			FindClose(hFinder);
			return false;
		}

		string fname;
		while (1) {
			fname = findPath + "\\" + fda.cFileName;
			if (fda.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
				if (strcmp(fda.cFileName, ".") != 0 && strcmp(fda.cFileName, "..") != 0 ) {
					// 递归删除子目录
					delFile(fname.c_str());
				}
			} else {
				DeleteFile(fname.c_str());
			}
			fret = FindNextFile(hf, &fda);
			if (fret == FALSE)
				break;
		}

		FindClose(hf);
		RemoveDirectory(findPath.c_str());

	} else {
		DeleteFile(findPath.c_str());
	}

	FindClose(hFinder);
	return true;
}


#else // Linux/Unix 平台特定实现

/**
 * @brief 判断文件或目录是否存在。
 * @param path 文件或目录的路径。
 * @return 如果存在返回 true，否则返回 false。
 */
bool FileUtil::exist(const string &path)
{
	struct stat statInfo;

	if (stat(path.c_str(), &statInfo) == 0)
	{
		return true;
	}

	return false;
}

/**
 * @brief 判断指定路径是否为目录。
 * @param path 路径。
 * @return 如果是目录返回 true，否则返回 false。
 */
bool FileUtil::isDirectory(const string & path)
{
	struct stat statInfo;

	if (stat(path.c_str(), &statInfo) == 0)
	{
		if (S_ISDIR(statInfo.st_mode))
		{
			return true;
		} else {
			return false;
		}
	}

	return false;
}

/**
 * @brief 如果目录不存在则创建目录。
 * @param path 要创建的目录路径。
 * @return 如果目录已存在或创建成功返回 true，否则返回 false。
 */
bool FileUtil::makeDirIfNoExist(const string & path)
{
	string::size_type sepPos = path.find_last_of("/");
	if (sepPos == string::npos)
	{
		return false;
	}

	string dirPath = path.substr(0, sepPos);
	if (exist(dirPath))
	{
		return true;
	}

	int ret = mkdir(dirPath.c_str(), S_IRWXU | S_IRGRP | S_IWGRP| S_IROTH | S_IWOTH);

	return 0 == ret ? true : false;
}

/**
 * @brief 删除文件或目录。
 * 如果是目录，则递归删除其内容。
 * @param filePath 要删除的文件或目录的路径。
 * @return 如果删除成功返回 true，否则返回 false。
 */
bool FileUtil::delFile(const string & filePath)
{
	if (!exist(filePath))
	{
		return false;
	}

	if (!isDirectory(filePath))
	{
		if (remove(filePath.c_str()) == 0)
		{
			return true;
		} else {
			return false;
		}
	}

	DIR* dir;
	struct dirent* ent;

	if ((dir = opendir(filePath.c_str())) == NULL)
	{
		return false;
	}

	while ((ent = readdir(dir)) != NULL)
	{
		if (strcmp(".", ent->d_name) == 0 || strcmp("..", ent->d_name) == 0)
		{
			continue;
		} else if (4 == ent->d_type || 8 == ent->d_type) {
			string fullSubPath;

			int len = filePath.size();
			if (filePath[len - 1] != '/')
			{
				fullSubPath.append(filePath).append("/").append(ent->d_name);
			} else {
				fullSubPath.append(filePath).append(ent->d_name);
			}

			delFile(fullSubPath);
		}
	}

	if (remove(filePath.c_str()) == 0)
	{
		return true;
	}

	return false;
}
#endif

string FileUtil::readFileAsString(const string & filePath)
{
	if (!exist(filePath))
	{
		return "";
	}

	fstream file(filePath.c_str(), ios::in | ios::binary);
	stringstream sstream;

	sstream << file.rdbuf();
	file.close();

	string str(sstream.str());
	sstream.clear();

	return str;
}

Buffer* FileUtil::readFileAsBuffer(const string &filePath)
{
	if (!exist(filePath))
	{
		return NULL;
	}

	fstream fs;
	fs.open(filePath.c_str(), ios::in | ios::binary);

	if (!fs.is_open())
	{
		return NULL;
	}

	fs.seekg(0, ios::end);
	int fileSize = fs.tellg();
	fs.seekg(0, ios::beg);

	Buffer* buffer = Buffer::alloc(fileSize);
	if (NULL == buffer)
	{
		fs.close();
		return NULL;
	}

	char* dataAddr = (char*) buffer->data();

	fs.read(dataAddr, fileSize);
	fs.close();

	return buffer;
}

string FileUtil::getCurrentTime()
{
	time_t t = time(NULL);
	tm* tm = localtime(&t);

	char buffer[20];
	sprintf(buffer, "%04d-%02d-%02d-%02d-%02d-%02d", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);

	return string(buffer);
}

FileUtil::DataFileHelper::DataFileHelper(const string& fileDir): mFileDir(fileDir)
{
	pthread_mutex_init(&mMutex, NULL);
}

FileUtil::DataFileHelper::~DataFileHelper()
{
	closeFile();
	pthread_mutex_destroy(&mMutex);
}

bool FileUtil::DataFileHelper::openReadFile(const string& filePath, bool inCurrentDir)
{
	string fullPath;

	if (inCurrentDir)
	{
		fullPath = mFileDir + filePath;
	} else {
		fullPath = filePath;
	}

	if (!exist(fullPath))
	{
		return false;
	}

	pthread_mutex_lock(&mMutex);
	if (mIn.is_open())
	{
		mIn.close();
	}

	mIn.open(fullPath.c_str(), ios::in | ios::binary);
	pthread_mutex_unlock(&mMutex);


	return true;
}

int FileUtil::DataFileHelper::read(char* buffer, int bufferLen)
{
	pthread_mutex_lock(&mMutex);
	if (!mIn.is_open())
	{
		pthread_mutex_unlock(&mMutex);
		return -1;
	}

	mIn.read(buffer, bufferLen);

	pthread_mutex_unlock(&mMutex);
	return mIn.gcount();
}

string FileUtil::DataFileHelper::readLine()
{
	pthread_mutex_lock(&mMutex);

	if (!mIn.is_open())
	{
		pthread_mutex_unlock(&mMutex);
		return "";
	}

	string line;
	getline(mIn, line);

	pthread_mutex_unlock(&mMutex);
	return line;
}

void FileUtil::DataFileHelper::rewindReadFile()
{
	pthread_mutex_lock(&mMutex);

	if (!mIn.is_open())
	{
		pthread_mutex_unlock(&mMutex);
		return;
	}

	if (mIn.eof())
	{
		mIn.clear();
	}

	mIn.seekg(0, ios::beg);

	pthread_mutex_unlock(&mMutex);
}

void FileUtil::DataFileHelper::closeReadFile()
{
	pthread_mutex_lock(&mMutex);

	if (mIn.is_open())
	{
		mIn.close();
	}
	pthread_mutex_unlock(&mMutex);
}

void FileUtil::DataFileHelper::createWriteFile(const string& filename, const string& suffix,
		bool append, bool inCurrentDir)
{
	string fullPath;

	if (inCurrentDir)
	{
		fullPath = mFileDir + filename + suffix;
	} else {
		fullPath = filename + suffix;
	}

	makeDirIfNoExist(fullPath);

	pthread_mutex_lock(&mMutex);

	if (append)
	{
		mOut.open(fullPath.c_str(), ios::out | ios::binary | ios::app);
	} else {
		mOut.open(fullPath.c_str(), ios::out | ios::binary);
	}
	pthread_mutex_unlock(&mMutex);
}

void FileUtil::DataFileHelper::createWriteFile(const string& suffix, bool append)
{
	string filename = getCurrentTime();
	createWriteFile(filename, suffix, append, true);
}

bool FileUtil::DataFileHelper::write(const char* data, int dataLen, int offset, int writeLen)
{
	if (offset + writeLen > dataLen)
	{
		return false;
	}

	pthread_mutex_lock(&mMutex);

	if (!mOut.is_open())
	{
		return false;
	}

	const char* start = data + offset;
	mOut.write(start, writeLen);
	mOut.flush();

	pthread_mutex_unlock(&mMutex);

	return true;
}

void FileUtil::DataFileHelper::closeWriteFile()
{
	pthread_mutex_lock(&mMutex);

	if (mOut.is_open())
	{
		mOut.flush();
		mOut.close();
	}

	pthread_mutex_unlock(&mMutex);
}

void FileUtil::DataFileHelper::closeFile()
{
	closeReadFile();
	closeWriteFile();
}


