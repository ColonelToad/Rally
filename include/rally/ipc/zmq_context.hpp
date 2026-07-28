#pragma once
#include <string>
#include <zmq.h>
#include <stdexcept>

namespace rally {
namespace ipc {

enum class SocketType {
    PUB,
    SUB,
    PUSH,
    PULL,
    REQ,
    REP
};

class ZmqContext {
public:
    ZmqContext();
    ~ZmqContext();
    
    // Delete copy/move constructors to strictly manage the singleton-like context
    ZmqContext(const ZmqContext&) = delete;
    ZmqContext& operator=(const ZmqContext&) = delete;

    void* get_raw_context() const { return context_; }

private:
    void* context_;
};

class ZmqSocket {
public:
    ZmqSocket(ZmqContext& context, SocketType type);
    ~ZmqSocket();

    void bind(const std::string& endpoint);
    void connect(const std::string& endpoint);
    
    void subscribe(const std::string& topic = "") {
        zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());
    }

    // ZERO-ALLOCATION TEMPLATE: Takes a pre-allocated struct by reference
    template <typename T>
    bool send(const T& message, int flags = 0) {
        int rc = zmq_send(socket_, &message, sizeof(T), flags);
        return rc == sizeof(T);
    }

    // ZERO-ALLOCATION TEMPLATE: Writes directly into a stack-allocated struct
    template <typename T>
    bool receive(T& out_message, int flags = 0) {
        int rc = zmq_recv(socket_, &out_message, sizeof(T), flags);
        return rc == sizeof(T);
    }

private:
    void unlink_if_ipc(const std::string& endpoint);
    
    void* socket_;
};

} // namespace ipc
} // namespace rally