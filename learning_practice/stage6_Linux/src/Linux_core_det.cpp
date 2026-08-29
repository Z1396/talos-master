#include <iostream>
#include <thread>

int main() {
    // 获取硬件支持的并发线程数
    unsigned int n = std::thread::hardware_concurrency();

    std::cout << "当前系统支持的并发线程数为: " << n << std::endl;

    if (n == 0) {
        std::cout << "无法确定或系统不支持并发查询。" << std::endl;
    }

    return 0;
}