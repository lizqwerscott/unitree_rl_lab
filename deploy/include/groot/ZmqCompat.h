#pragma once

// Version-independent ZeroMQ helpers.
//
// cppzmq < 4.7 only provides `setsockopt(int, T)` while cppzmq >= 4.7
// deprecates it in favour of `socket.set(zmq::sockopt::...)`. Setting the
// options through the stable libzmq C API avoids the version split.

#include <zmq.hpp>

namespace groot {
namespace zmq_detail {

template <typename T>
auto socket_handle(T& socket, int) -> decltype(socket.handle()) {
    return socket.handle();
}

template <typename T>
void* socket_handle(T& socket, long) {
    return static_cast<void*>(socket);
}

inline void set_sockopt_int(zmq::socket_t& socket, int option, int value) {
    zmq_setsockopt(socket_handle(socket, 0), option, &value, sizeof(value));
}

}  // namespace zmq_detail
}  // namespace groot
