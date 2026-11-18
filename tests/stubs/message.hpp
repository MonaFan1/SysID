#pragma once
#include "queue/spsc_queue.hpp"
#include "timebase.hpp"
#include <cstdint>
#include <functional>
#include <vector>
namespace LibXR {
template <class T> struct TopicHooks {
  static inline std::vector<std::function<void(uint64_t, const T &)>> sinks;
};
struct Topic {
  template <class T> struct Message {
    uint64_t timestamp;
    T data;
  };
  class QueuedSubscriber {
  public:
    template <class T>
    QueuedSubscriber(const char *, SPSCQueue<Message<T>> &q) {
      TopicHooks<T>::sinks.push_back([&q](uint64_t stamp, const T &data) {
        q.Push(Message<T>{stamp, data});
      });
    }
  };
  template <class T> struct ASyncSubscriber {
    static inline T value{};
    static inline uint64_t sequence = 0;
    uint64_t seen = 0;
    explicit ASyncSubscriber(const char *) {}
    void StartWaiting() { seen = sequence; }
    bool Available() { return sequence != seen; }
    T GetData() { return value; }
    static void Publish(T data) {
      value = data;
      for (auto &sink : TopicHooks<T>::sinks)
        sink(Timebase::now, data);
      ++sequence;
    }
  };
};
} // namespace LibXR
