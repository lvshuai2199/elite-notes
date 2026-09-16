#ifndef __PRIMARY_PORT_HPP__
#define __PRIMARY_PORT_HPP__

#include "RobotException.hpp"

#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <functional>

#define ITEM_MAX_DATA_BUFF (8)

class RobotState {
   public:
    struct Item {
        std::string name;
        int size;
        char data_buff[ITEM_MAX_DATA_BUFF];
    };

    enum ConnectionState { DISCONNECTED = 0, CONNECTED = 1 };

    void setExceptionCallback(const std::function<void(const RobotException&)>& callback);

    bool loadConfigure(const std::string& file_path);
    bool connect(const std::string& ip, int port);
    bool isConnect();
    void disconnect();
    int sendScript(const std::string& script);

    bool findItem(const std::string& name, Item* item);

    template <typename T>
    T getItemData(const std::string& name, T* data) {
        Item item;
        if (!findItem(name, &item)) {
            return (T)0;
        }
        T result = (T)0;
        if (item.size == sizeof(T)) {
            memcpy(&result, item.data_buff, sizeof(T));
            if (data) {
                *data = result;
            }
        }
        return result;
    }

    RobotState();
    ~RobotState();

    // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
    RobotState(const RobotState&) = delete;
    RobotState& operator=(const RobotState&) = delete;
    RobotState(RobotState&&) = delete;
    RobotState& operator=(RobotState&&) = delete;

   private:
    class LoadFile;

    std::unique_ptr<std::thread> recv_thread_handle;
    bool recv_thread_keep_alive;

    typedef std::unique_lock<std::shared_timed_mutex> WriteLock;
    typedef std::shared_lock<std::shared_timed_mutex> ReadLock;
    std::shared_timed_mutex packages_mutex;
    typedef std::unordered_map<std::string, Item> Packages;
    typedef std::vector<std::string> PackagesOrder;
    Packages packages;
    PackagesOrder packages_order;
    std::function<void(const RobotException&)> robot_exception_callback;

    std::mutex socket_instance_mutex;
    struct Sokcet;
    std::unique_ptr<Sokcet> robot_socket;

    ConnectionState connection_state;

    static void flipBytes(void* data, int size);
    void updateRobotState(uint8_t* recv_buff);
    int socketRecv(void* out_msg, int size);
    bool abortMessageRecv(int size);
    void recv_thread();
    bool convertByteStream(const uint8_t* bytes, uint32_t bytes_len);
};

class RobotState::LoadFile {
   private:
    static bool analyzeItem(const std::string& line, RobotState::Item* item, std::string& prefix);

   public:
    static bool loadFile(const std::string& file_path, RobotState::Packages* packages, RobotState::PackagesOrder* packages_order);
    static bool insertItem(const RobotState::Item& item, RobotState::Packages* packages, RobotState::PackagesOrder* packages_order);
};

#endif