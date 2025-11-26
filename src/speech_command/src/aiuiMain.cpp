/**
 * @file aiuiMain.cpp
 * @brief AIUI 语音交互主程序。
 * 该文件包含了 ROS 节点初始化、AIUI 语音识别结果处理、串口通信以及音频播放等功能。
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <AIUITester.h>
#include <signal.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <thread>
#include <iostream>
#include <mutex>
#include "jsoncpp/json/json.h"

char pp11_[3000]; /**< @brief 存储路径的缓冲区。 */
using namespace std;
vector<string> third_line; /**< @brief 存储配置文件中第三列数据的向量。 */
bool get_request_test = false; /**< @brief 测试请求标志位。 */
#define TIMEOUT 10 /**< @brief 超时时间定义。 */

/**
 * @brief 读取用户配置文件数据，并保存到内存中。
 * 配置文件格式为：问题1|问题2|...:协议:文档路径
 * @param config_txt 配置文件的路径。
 * @return int 0 表示成功，-1 表示文件无法打开。
 */
int LoadUserConfig(string config_txt)
{
	std::ifstream fd(config_txt.c_str());
	if (!fd.is_open())
	{
		cout << "无法打开用户配置文件，请确保其可读。" << endl;
		return -1;
	}
	string line;
	while (std::getline(fd, line))
	{
		vector<string> vstr;
		// 使用 ':' 分割行，预期得到三部分：问题、协议、文档路径
		boost::split(vstr, line, boost::is_any_of(":"));
		if (vstr.size() != 3)
		{
			// 如果分割后不是三部分，则跳出循环，认为文件格式不正确
			break;
		}
		vector<string> questions;

		third_line.push_back(vstr[2]); // 将文档路径添加到 third_line 向量中
		//cout<<"vstr_third_line:"<<vstr[2]<<endl;
		// 使用 '|' 分割问题部分，得到多个问题
		boost::split(questions, vstr[0], boost::is_any_of("|"));
		for (vector<string>::iterator it = questions.begin(); it != questions.end(); ++it)
		{
			// 将问题和对应的协议、文档路径存储到全局 map 中
			QA_list_.insert(std::pair<std::string, std::string>(*it, vstr[1]));
			QA_list_doc.insert(std::pair<std::string, std::string>(*it, vstr[2]));
		}
	}
	return 0;
}

/**
 * @brief 在用户列表中查找识别到的结果，返回对应的协议数据。
 * 该函数会处理识别结果，移除句号，然后查找匹配的关键词并返回对应的协议。
 * @param recognise_result 语音识别的结果字符串。
 * @return std::string 匹配到的协议字符串，如果未找到则返回空字符串。
 */
std::string FindKeyWords(std::string recognise_result)
{
	std::map<std::string, std::string>::iterator it;
	// for(it=QA_list_.begin();it!=QA_list_.end();it++){
	// 	if (recognise_result.find(it->first)!= string::npos){
	// 		return it->second;
	// 	} 
	// }
	// return "";
	// 移除识别结果中的中文句号
	auto find_result_ = recognise_result.find("。");
	if (find_result_ != string::npos)
	{
		recognise_result.erase(find_result_, 1);
	}
	// 移除识别结果中的英文句号
	auto find_result_1 = recognise_result.find(".");
	if (find_result_1 != string::npos)
	{
		recognise_result.erase(find_result_1, 1);
	}
	// 在 QA_list_ 中查找匹配的关键词
	it = QA_list_.find(recognise_result);
	if (it != QA_list_.end())
	{
		return it->second; // 返回对应的协议
	}
	else
	{
		return ""; // 未找到则返回空字符串
	}
}

/**
 * @brief 在用户列表中查找识别到的结果，返回对应的文档路径。
 * 该函数会处理识别结果，移除句号，然后查找匹配的关键词并返回对应的文档路径。
 * @param recognise_result 语音识别的结果字符串。
 * @return std::string 匹配到的文档路径字符串，如果未找到则返回空字符串。
 */
std::string FindDocument(std::string recognise_result)
{
	std::map<std::string, std::string>::iterator its;
	// for(it=QA_list_.begin();it!=QA_list_.end();it++){
	// 	if (recognise_result.find(it->first)!= string::npos){
	// 		return it->second;
	// 	}
	// }
	// return "";
	// 移除识别结果中的中文句号
	auto find_result_ = recognise_result.find("。");
	if (find_result_ != string::npos)
	{
		recognise_result.erase(find_result_, 1);
	}
	// 在 QA_list_doc 中查找匹配的关键词
	its = QA_list_doc.find(recognise_result);
	if (its != QA_list_doc.end())
	{
		return its->second; // 返回对应的文档路径
	}
	else
	{
		return ""; // 未找到则返回空字符串
	}
}

/**
 * @brief 串口写入函数。
 * 该函数目前为空实现，预留用于串口数据写入。
 */
void write_serial()
{
/*
	

	const int numBytes = str.size() / 2;
	unsigned char *bytes = new unsigned char[numBytes];
	for (int i = 0; i < numBytes; ++i)
	{
		std::string twoChars = str.substr(2 * i, 2);
		int byte;
		std::stringstream ss(twoChars);
		ss >> std::hex >> byte;
		bytes[i] = byte;
	}
	//_serial.flush();
	_serial.write(bytes, 8);
*/
	//_serial.read(buffer, 8);
	//printf("123, =%d\n",buffer);
}

/**
 * @brief ROS 数据发送主函数。
 * 该函数初始化 ROS 节点，创建 ROS 发布者，并循环发布语音识别的问题、答案和角度信息。
 * 同时处理本地和云端对话，并根据识别结果执行相应的命令或播放音频。
 * @param argc 命令行参数计数。
 * @param argv 命令行参数数组。
 * @return int 始终返回 0。
 */
int data_send(int argc, char **argv)
{
	ros::init(argc, argv, "publisher_Node"); // 初始化 ROS 节点
	ros::NodeHandle n; // 创建 ROS 节点句柄
	// 创建 ROS 发布者，用于发布问题、答案和角度信息
	ros::Publisher pub_question = n.advertise<std_msgs::String>("/question", 10);
	ros::Publisher pub_answer = n.advertise<std_msgs::String>("/answer", 10);
	ros::Publisher pub_angle = n.advertise<std_msgs::Int32>("/angle", 10);
	string ros_package_path1 = ros::package::getPath("speech_command"); // 获取 ROS 包路径
	package_path1 = const_cast<char *>(ros_package_path1.c_str()); // 将包路径转换为 char* 类型
        
	while (ros::ok()) // 循环直到 ROS 关闭
	{	
		int num=0;
		int angle_str(angle);
        std_msgs::Int32 msg2;
		msg2.data = angle;
		pub_angle.publish(msg2); // 发布角度信息
			
		if ((sign_conversation_cloud == 1) || (sign_conversation_local == 1)) // 如果有云端或本地对话标志
		{
			string question_str(question); // 获取问题字符串
			string answer_str(answer); // 获取答案字符串
			int angle_str(angle);
            cout << "角度:\t" << angle_str << "\n";
			// 发送问题
			std_msgs::String msg1;
			msg1.data = question_str;
			pub_question.publish(msg1);

			// 发送答案
			std_msgs::String msg3;
			msg3.data = answer_str;
			while(num<10){
				pub_answer.publish(msg3); // 循环发布答案，确保接收
				num++;
			}
			// 串口操作
			//cout<<"456456645445454545"<<endl;
			cout << "问题:\t" << question_str << "\n"
				 << "答案:\t" << answer_str << endl;
			if (sign_conversation_local == 1) // 如果是本地对话
			{
				string command_ = FindKeyWords(question_str); // 查找关键词对应的命令
				string document = FindDocument(question_str); // 查找关键词对应的文档路径
				cout << "下发协议:\t" << command_ << endl;
				strcpy(pp11_, package_path1); // 复制包路径
				string  wav = string((char*)pp11_);
				std::string wav_path1 = wav.append(document); // 拼接音频文件路径
				std::string command1 = "play "+wav_path1; // 构建播放命令
				system(command1.c_str()); // 执行播放命令
				//_serial.write(command_); // 预留串口写入
			}

			sign_conversation_cloud = 0; // 重置云端对话标志
			sign_conversation_local = 0; // 重置本地对话标志
		}
		/*
		if (sign_angle == true)
		{
			//发送角度
			std_msgs::Int32 msg2;
			msg2.data = angle;
			pub_angle.publish(msg2);
			sign_angle = false;
		}*/
		ros::spinOnce(); // 处理 ROS 事件
	}
}
int main(int argc, char **argv)
{
	//print("%s",get_software_version());
	cout<<get_software_version()<<endl;
	try
	{
		_serial.setPort(DEV_ID);
		_serial.setBaudrate(BAUD_RATE);
		_serial.setFlowcontrol(serial::flowcontrol_none);
		_serial.setParity(serial::parity_none); //default is parity_none
		_serial.setStopbits(serial::stopbits_one);
		_serial.setBytesize(serial::eightbits);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}

	if (_serial.isOpen())
	{
		sleep(1/10);//ros::Duration(0.1).sleep(); // wait 0.1s
		_serial.flush();			// wait 0.1s
		ROS_INFO_STREAM("port initial successfully------");
	}
	string ros_package_path = ros::package::getPath("speech_command");
	package_path = const_cast<char *>(ros_package_path.c_str());
	// 相对位置 ->  绝对位置
	char pp_[3000], pp1_[3000], pp2_[3000], pp3_[3000], pp4_[3000], pp5_[3000], pp6_[3000], pp7_[3000], pp8_[3000], pp9_[3000], pp10_[3000];
	strcpy(pp_, package_path);
	CFG_FILE_PATH = strcat(pp_, CFG_FILE_PATH);

	strcpy(pp1_, package_path);
	SOURCE_FILE_PATH = strcat(pp1_, SOURCE_FILE_PATH);

	strcpy(pp2_, package_path);
	GRAMMAR_FILE_PATH = strcat(pp2_, GRAMMAR_FILE_PATH);

	strcpy(pp3_, package_path);
	TEST_AUDIO_PATH = strcat(pp3_, TEST_AUDIO_PATH);

	strcpy(pp4_, package_path);
	LOG_DIR = strcat(pp4_, LOG_DIR);

	strcpy(pp5_, package_path);
	CONFIG_FILE_PATH = strcat(pp5_, CONFIG_FILE_PATH);

	strcpy(pp6_, package_path);
	PCM_FILE_PATH = strcat(pp6_, PCM_FILE_PATH);

	strcpy(pp7_, package_path);
	ORIPCM_FILE_PATH = strcat(pp7_, ORIPCM_FILE_PATH);

	strcpy(pp8_, package_path);

	strcpy(pp9_, package_path);
	NO_INTERNET_RESPONSE_WAV = strcat(pp9_, NO_INTERNET_RESPONSE_WAV);



	// 加载用户离线语义表
	string user_config_path = ros_package_path + USER_CONFIG_PATH;
	LoadUserConfig(user_config_path);
	AIUITester t;



	std::thread t1(std::bind(&AIUITester::test,&t));
        std::thread t2(data_send,argc, argv);
	
	

	t1.join();
	t2.join();



	t.destory();

	return 0;
}
