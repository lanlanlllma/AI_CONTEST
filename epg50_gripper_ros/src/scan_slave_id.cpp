#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>

class SlaveIDScanner {
private:
    int serial_port;
    const uint16_t READ_REG_START = 0x07D0; // EPG50读寄存器首地址

    // CRC16计算（Modbus RTU）
    uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // 发送Modbus命令并接收响应
    bool send_command(const std::vector<uint8_t>& command, std::vector<uint8_t>& response, int timeout_ms = 200) {
        response.clear();

        // 清空接收缓冲区
        tcflush(serial_port, TCIFLUSH);

        if (write(serial_port, command.data(), command.size()) < 0) {
            return false;
        }

        // 等待响应
        auto start = std::chrono::steady_clock::now();
        uint8_t buffer[256];

        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
            ssize_t len = read(serial_port, buffer, sizeof(buffer));
            if (len > 0) {
                response.insert(response.end(), buffer, buffer + len);

                // 检查是否收到完整响应
                if (is_response_complete(response)) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return !response.empty();
    }

    bool is_response_complete(const std::vector<uint8_t>& response) {
        if (response.size() < 4)
            return false;

        uint8_t function_code = response[1];

        // 读寄存器响应 (0x03)
        if (function_code == 0x03) {
            if (response.size() < 3)
                return false;
            uint8_t byte_count = response[2];
            return static_cast<int>(response.size()) >= byte_count + 5;
        }

        return false;
    }

    // 验证CRC
    bool verify_crc(const std::vector<uint8_t>& response) {
        if (response.size() < 4)
            return false;

        size_t data_len         = response.size() - 2;
        uint16_t received_crc   = response[data_len] | (response[data_len + 1] << 8);
        uint16_t calculated_crc = crc16(response.data(), data_len);

        return received_crc == calculated_crc;
    }

public:
    SlaveIDScanner(const std::string& port) {
        serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_port < 0) {
            throw std::runtime_error("无法打开串口: " + port + " - " + strerror(errno));
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port, &tty) != 0) {
            close(serial_port);
            throw std::runtime_error("获取串口属性失败");
        }

        // 配置串口参数（115200 8N1）
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            close(serial_port);
            throw std::runtime_error("设置串口属性失败");
        }

        std::cout << "串口 " << port << " 已打开，配置为 115200 8N1" << std::endl;
    }

    ~SlaveIDScanner() {
        if (serial_port >= 0) {
            close(serial_port);
        }
    }

    // 尝试读取指定slave_id的状态寄存器
    bool try_slave_id(uint8_t slave_id) {
        // 构造Modbus RTU读寄存器命令
        // 功能码 0x03: 读保持寄存器
        // 寄存器地址: 0x07D0 (EPG50状态寄存器起始地址)
        // 寄存器数量: 0x0004 (读取4个寄存器)
        std::vector<uint8_t> cmd = {
            slave_id,
            0x03, // 功能码: 读保持寄存器
            static_cast<uint8_t>((READ_REG_START >> 8) & 0xFF),
            static_cast<uint8_t>(READ_REG_START & 0xFF),
            0x00,
            0x04 // 读取4个寄存器
        };

        // 添加CRC校验
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);

        std::vector<uint8_t> response;
        if (send_command(cmd, response)) {
            // 验证响应
            if (response.size() >= 4 && response[0] == slave_id && response[1] == 0x03) {
                if (verify_crc(response)) {
                    return true;
                }
            }
        }

        return false;
    }

    // 扫描从 start_id 到 end_id 的所有 slave ID
    void scan_range(uint8_t start_id = 0x01, uint8_t end_id = 0xFF) {
        std::cout << "\n开始扫描 Slave ID 范围: 0x" << std::hex << static_cast<int>(start_id) << " - 0x"
                  << static_cast<int>(end_id) << std::dec << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

        std::vector<uint8_t> found_ids;

        for (int id = start_id; id <= end_id; ++id) {
            uint8_t slave_id = static_cast<uint8_t>(id);

            // 显示进度
            if (id % 16 == 0 || id == start_id) {
                std::cout << "\n扫描进度: 0x" << std::hex << id << std::dec << " (" << id << "/"
                          << static_cast<int>(end_id) << ") ";
            } else {
                std::cout << "." << std::flush;
            }

            if (try_slave_id(slave_id)) {
                found_ids.push_back(slave_id);
                std::cout << "\n✓ 发现设备! Slave ID = 0x" << std::hex << static_cast<int>(slave_id)
                          << " (十进制: " << std::dec << static_cast<int>(slave_id) << ")" << std::endl;
            }

            // 避免总线拥塞，每次扫描后稍作延迟
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "\n扫描完成!" << std::endl;

        if (found_ids.empty()) {
            std::cout << "❌ 未找到任何响应的设备" << std::endl;
            std::cout << "\n可能的原因:" << std::endl;
            std::cout << "  1. 设备未连接或未通电" << std::endl;
            std::cout << "  2. 串口设备路径不正确" << std::endl;
            std::cout << "  3. 波特率不匹配（当前: 115200）" << std::endl;
            std::cout << "  4. 串口权限不足（尝试: sudo 或加入 dialout 组）" << std::endl;
        } else {
            std::cout << "✓ 找到 " << found_ids.size() << " 个设备:" << std::endl;
            for (auto id: found_ids) {
                std::cout << "  - Slave ID: 0x" << std::hex << static_cast<int>(id) << " (十进制: " << std::dec
                          << static_cast<int>(id) << ")" << std::endl;
            }

            std::cout << "\n使用方法:" << std::endl;
            std::cout << "  ros2 launch epg50_gripper_ros launch.py default_slave_id:="
                      << static_cast<int>(found_ids[0]) << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    std::string port = "/dev/ttyACM0";
    uint8_t start_id = 0x01;
    uint8_t end_id   = 0xFF;

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            port = argv[++i];
        } else if ((arg == "-s" || arg == "--start") && i + 1 < argc) {
            start_id = static_cast<uint8_t>(std::stoi(argv[++i], nullptr, 0));
        } else if ((arg == "-e" || arg == "--end") && i + 1 < argc) {
            end_id = static_cast<uint8_t>(std::stoi(argv[++i], nullptr, 0));
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "EPG50 Gripper Slave ID 扫描工具" << std::endl;
            std::cout << "\n用法: " << argv[0] << " [选项]" << std::endl;
            std::cout << "\n选项:" << std::endl;
            std::cout << "  -p, --port <串口>     指定串口设备 (默认: /dev/ttyACM0)" << std::endl;
            std::cout << "  -s, --start <ID>      起始 Slave ID (默认: 1, 支持十进制或0x十六进制)" << std::endl;
            std::cout << "  -e, --end <ID>        结束 Slave ID (默认: 255)" << std::endl;
            std::cout << "  -h, --help            显示此帮助信息" << std::endl;
            std::cout << "\n示例:" << std::endl;
            std::cout << "  " << argv[0] << "                              # 扫描默认端口全部ID" << std::endl;
            std::cout << "  " << argv[0] << " -p /dev/ttyUSB0              # 指定串口" << std::endl;
            std::cout << "  " << argv[0] << " -s 1 -e 32                   # 仅扫描 1-32" << std::endl;
            std::cout << "  " << argv[0] << " -s 0x01 -e 0x20              # 十六进制表示" << std::endl;
            return 0;
        }
    }

    try {
        std::cout << "═══════════════════════════════════════════════" << std::endl;
        std::cout << "   EPG50 Gripper Slave ID 扫描工具" << std::endl;
        std::cout << "═══════════════════════════════════════════════" << std::endl;
        std::cout << "配置:" << std::endl;
        std::cout << "  串口: " << port << std::endl;
        std::cout << "  波特率: 115200 8N1" << std::endl;
        std::cout << "  扫描范围: 0x" << std::hex << static_cast<int>(start_id) << " - 0x" << static_cast<int>(end_id)
                  << std::dec << std::endl;

        SlaveIDScanner scanner(port);
        scanner.scan_range(start_id, end_id);

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << std::endl;
        std::cerr << "\n提示:" << std::endl;
        std::cerr << "  1. 检查设备是否连接: ls -l /dev/ttyACM* /dev/ttyUSB*" << std::endl;
        std::cerr << "  2. 检查串口权限: sudo chmod 666 " << port << std::endl;
        std::cerr << "  3. 或将用户加入 dialout 组: sudo usermod -a -G dialout $USER" << std::endl;
        std::cerr << "  4. 查看内核日志: dmesg | tail -n 20 | grep -i tty" << std::endl;
        return 1;
    }
}
