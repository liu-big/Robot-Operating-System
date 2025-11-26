/**
 * @file serial_port.cpp
 * @brief 串口通信功能实现文件。
 * 该文件包含了串口的打开、关闭、设置、初始化、接收和发送等基本操作函数。
 */

#include <serial_port.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

/**
 * @brief 打开串口。
 * @param fd 串口文件描述符，传入时通常为-1，函数成功打开串口后会返回新的文件描述符。
 * @param port 串口设备路径，例如 "/dev/ttyS0" 或 "/dev/ttyUSB0"。
 * @return 成功返回文件描述符，失败返回 -1。
 */
int UART_Open(int fd, char *port)
{
	// O_RDWR: 读写模式
	// O_NOCTTY: 不作为控制终端
	// O_NDELAY: 非阻塞模式，open 不会等待 DCD 信号
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd < 0)
	{
		perror("无法打开串口");
		return -1;
	}
	
	// 恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, 0) < 0)    
    {
        printf("fcntl 失败!\n");    
        return -1;    
    }         
    else    
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));    
    }
    // 测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))    
    {
        printf("标准输入不是终端设备\n");    
        return -1;    
    }    
    else    
    {
        printf("isatty 成功!\n");    
    }
    printf("文件描述符 fd->open=%d\n",fd);    
	
    return fd; 
}

/**
 * @brief 关闭串口。
 * @param fd 串口文件描述符。
 */
void UART_Close(int fd)
{
	close(fd);
}

/**
 * @brief 设置串口数据位、停止位和校验位。
 * @param fd 串口文件描述符。
 * @param speed 串口波特率，例如 115200。
 * @param flow_ctrl 数据流控制，0: 不使用流控制, 1: 硬件流控制, 2: 软件流控制。
 * @param databits 数据位，取值为 5, 6, 7 或 8。
 * @param stopbits 停止位，取值为 1 或 2。
 * @param parity 校验类型，取值为 'n'/'N' (无校验), 'o'/'O' (奇校验), 'e'/'E' (偶校验), 's'/'S' (空格校验)。
 * @return 成功返回 0，失败返回 -1。
 */
int UART_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity)
{
	int   i;    
    int   status;    
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};    
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};    
             
    struct termios options;    
       
    /* tcgetattr(fd,&options) 获取与 fd 指向对象相关的参数，并将它们保存到 options 中。
     * 该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1。
     */
    if(tcgetattr(fd,&options) != 0)    
    {
        perror("设置串口参数 1 失败");        
        return -1;     
    }    
      
    // 设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)    
    {
        if  (speed == name_arr[i])    
        {
            cfsetispeed(&options, speed_arr[i]);     
            cfsetospeed(&options, speed_arr[i]);      
        }    
    }         
       
    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;    
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;    
      
    // 设置数据流控制
    switch(flow_ctrl)    
    {
        case 0 :// 不使用流控制
              options.c_cflag &= ~CRTSCTS;    
              break;       
        case 1 :// 使用硬件流控制
              options.c_cflag |= CRTSCTS;    
              break;    
        case 2 :// 使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;    
              break;    
    }    
    // 设置数据位
    // 屏蔽其他标志位
    options.c_cflag &= ~CSIZE;    
    switch (databits)    
    {
        case 5:    
                 options.c_cflag |= CS5;    
                 break;    
        case 6:    
                 options.c_cflag |= CS6;    
                 break;    
        case 7:        
                 options.c_cflag |= CS7;    
                 break;    
        case 8:        
                 options.c_cflag |= CS8;    
                 break;      
        default:       
                 fprintf(stderr,"不支持的数据位大小\n");    
                 return -1;     
    }    
    // 设置校验位
    switch (parity)    
    {
        case 'n':    
        case 'N': // 无奇偶校验位。
                 options.c_cflag &= ~PARENB;     
                 options.c_iflag &= ~INPCK;        
                 break;     
        case 'o':      
        case 'O':// 设置为奇校验        
                 options.c_cflag |= (PARODD | PARENB);     
                 options.c_iflag |= INPCK;                 
                 break;     
        case 'e':     
        case 'E':// 设置为偶校验      
                 options.c_cflag |= PARENB;           
                 options.c_cflag &= ~PARODD;           
                 options.c_iflag |= INPCK;          
                 break;    
        case 's':    
        case 'S': // 设置为空格校验
                 options.c_cflag &= ~PARENB;    
                 options.c_cflag &= ~CSTOPB;    
                 break;     
        default:      
                 fprintf(stderr,"不支持的校验类型\n");        
                 return -1;     
    }     
    // 设置停止位
    switch (stopbits)    
    {
        case 1:       
                 options.c_cflag &= ~CSTOPB; break;     
        case 2:       
                 options.c_cflag |= CSTOPB; break;    
        default:       
                       fprintf(stderr,"不支持的停止位\n");     
                       return -1;    
    }    
       
    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;    
      
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    //options.c_lflag &= ~(ISIG | ICANON);    
       
    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */      
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */    
       
    // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);    
       
    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)      
    {
        perror("串口设置失败!\n");      
        return -1;     
    }    
    return 0;
}

/**
 * @brief 串口初始化。
 * 该函数调用 UART_Set 函数来设置串口的默认参数：波特率 115200，无流控制，8 位数据位，1 位停止位，无校验。
 * @param fd 串口文件描述符。
 * @param speed 串口速度 (在此函数中固定为 115200)。
 * @param flow_ctrl 数据流控制 (在此函数中固定为 0，即无流控制)。
 * @param databits 数据位 (在此函数中固定为 8)。
 * @param stopbits 停止位 (在此函数中固定为 1)。
 * @param parity 校验类型 (在此函数中固定为 'N'，即无校验)。
 * @return 成功返回 0，失败返回 -1。
 */
int UART_Init(int fd, int speed, int flow_ctrl, int databits, int stopbits, char parity) 
{
	// 调用 UART_Set 设置串口参数，如果设置失败则返回 -1
	if(UART_Set(fd, 115200, 0, 8, 1, 'N') == -1)
		return -1;
	else 
		return 0;
}


/**
 * @brief 接收串口数据。
 * 使用 select 机制等待串口数据，并在数据到达时读取指定长度的数据。
 * @param fd 文件描述符。
 * @param rcv_buf 接收串口中数据存入的缓冲区。
 * @param data_len 期望接收的数据长度。
 * @return 成功返回实际读取的字节数，失败返回 -1。
 */
int UART_Recv(int fd, char *rcv_buf, int data_len)
{
	int len, fs_sel;    
    fd_set fs_read;    
       
    struct timeval time;    
       
    FD_ZERO(&fs_read); // 清空文件描述符集
    FD_SET(fd, &fs_read); // 将串口文件描述符加入到集合中
       
    time.tv_sec = 10; // 设置超时时间为 10 秒
    time.tv_usec = 0;    
       
    // 使用 select 实现串口的多路通信，等待数据可读或超时
    fs_sel = select(fd+1, &fs_read, NULL, NULL, &time);    
    printf("select 返回值 fs_sel = %d\n",fs_sel);    
    if(fs_sel)    
    {
        len = read(fd, rcv_buf, data_len); // 从串口读取数据
        return len;    
    }    
    else       
        return -1;    
}
/**
 * @brief 发送数据到串口。
 * 将指定缓冲区的数据写入串口。
 * @param fd 文件描述符。
 * @param send_buf 存放串口发送数据的缓冲区。
 * @param data_len 期望发送的数据长度。
 * @return 成功返回实际写入的字节数，失败返回 -1。
 */
int UART_Send(int fd, char *send_buf, int data_len)
{
	int len = 0;    
    
    len = write(fd, send_buf, data_len); // 写入数据到串口
    if (len == data_len )    
    {
        printf("发送数据为: %s\n", send_buf);  
        return len;    
    }         
    else       
    {
        tcflush(fd, TCOFLUSH); // 刷新输出缓冲区
        return -1;    
    }  
}



