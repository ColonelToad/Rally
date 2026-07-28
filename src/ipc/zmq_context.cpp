#include "rally/ipc/zmq_context.hpp"
#include <cstdio> // for std::remove
#include <stdexcept>
#include <iostream>

namespace rally {
namespace ipc {

// --- ZmqContext Implementation ---

ZmqContext::ZmqContext() {
    context_ = zmq_ctx_new();
    if (!context_) {
        throw std::runtime_error("Failed to initialize ZeroMQ context");
    }
}

ZmqContext::~ZmqContext() {
    if (context_) {
        zmq_ctx_term(context_);
    }
}

// --- ZmqSocket Implementation ---

ZmqSocket::ZmqSocket(ZmqContext& context, SocketType type) {
    int zmq_type;
    switch (type) {
        case SocketType::PUB:  zmq_type = ZMQ_PUB; break;
        case SocketType::SUB:  zmq_type = ZMQ_SUB; break;
        case SocketType::PUSH: zmq_type = ZMQ_PUSH; break;
        case SocketType::PULL: zmq_type = ZMQ_PULL; break;
        case SocketType::REQ:  zmq_type = ZMQ_REQ; break;
        case SocketType::REP:  zmq_type = ZMQ_REP; break;
        default: throw std::invalid_argument("Invalid socket type");
    }

    socket_ = zmq_socket(context.get_raw_context(), zmq_type);
    if (!socket_) {
        throw std::runtime_error("Failed to create ZeroMQ socket");
    }
}

ZmqSocket::~ZmqSocket() {
    if (socket_) {
        zmq_close(socket_);
    }
}

void ZmqSocket::unlink_if_ipc(const std::string& endpoint) {
    if (endpoint.find("ipc://") == 0) {
        std::string path = endpoint.substr(6);
        std::remove(path.c_str()); 
    }
}

void ZmqSocket::bind(const std::string& endpoint) {
    unlink_if_ipc(endpoint);
    int rc = zmq_bind(socket_, endpoint.c_str());
    if (rc != 0) {
        throw std::runtime_error("Failed to bind ZeroMQ socket to " + endpoint);
    }
}

void ZmqSocket::connect(const std::string& endpoint) {
    int rc = zmq_connect(socket_, endpoint.c_str());
    if (rc != 0) {
        throw std::runtime_error("Failed to connect ZeroMQ socket to " + endpoint);
    }
}

} // namespace ipc
} // namespace rally