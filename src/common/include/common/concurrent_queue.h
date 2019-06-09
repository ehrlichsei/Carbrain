#ifndef CONCURRENT_QUEUE_H
#define CONCURRENT_QUEUE_H

#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <pthread.h>

namespace common {
namespace realtime {

/*!
 * \brief The ConcurrentQueue class implements a thread-safe blocking queue.
 * It has a fixed size, does not allocate or deallocate memory and can be
 * used in real-time code.
 * If the queue is full and a new element is pushed to the queue, the first
 * element is discarded.
 * All operations are O(1).
 * The ConcurrentQueue is internally implemented as Ring Buffer.
 * All methods are real-time safe, no additional memory is allocated and it
 * is protected using PTHREAD_PRIO_INHERIT mutexes.
 */
template <typename Data, int Capacity>
class ConcurrentQueue {
 private:
  Data elements[Capacity];
  int head_index;
  int size;

  mutable pthread_mutex_t mutex;
  pthread_mutexattr_t mutex_attr;
  pthread_cond_t condition_variable;

 public:
  /*!
   * \brief CuncurrentQueue setups an empty() queue.
   */
  ConcurrentQueue() {
    head_index = size = 0;

    pthread_cond_init(&condition_variable, nullptr);

    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&mutex, &mutex_attr);
  }

  ~ConcurrentQueue() {
    pthread_mutexattr_destroy(&mutex_attr);
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&condition_variable);
  }

  /*!
   * \brief push appends an element to the queue.
   * \param data the element ot append.
   */
  void push(Data const& data) {
    pthread_mutex_lock(&mutex);
    int next_index = (head_index + size) % Capacity;
    if (size == Capacity) {
      head_index = (head_index + 1) % Capacity;
    } else {
      size++;
    }
    elements[next_index] = data;
    pthread_mutex_unlock(&mutex);
    pthread_cond_signal(&condition_variable);
  }

  /*!
   * \brief empty returns whether the queue is empty.
   */
  bool empty() const {
    pthread_mutex_lock(&mutex);
    const bool is_empty = size == 0;
    pthread_mutex_unlock(&mutex);
    return is_empty;
  }

  /*!
   * \brief full returns whether the queue is full.
   */
  bool full() const {
    pthread_mutex_lock(&mutex);
    bool is_full = size == Capacity;
    pthread_mutex_unlock(&mutex);
    return is_full;
  }

  /*!
   * \brief pop removes the last element of the queue.
   * \b Note: if the queue is empty(), pop() blocks until it is not empty()
   * anymore.
   * \param result the data of the removed element.
   * \return wether the operation has been interrupted.
   */
  bool pop(Data* result) {
    // lock the mutex in this scope and give the thread mutex and
    // contition_variable in order to be interruptable during waiting for
    // condition.
    boost::detail::interruption_checker interruption_guard(&mutex, &condition_variable);
    while (size == 0) {
      if (boost::this_thread::interruption_requested()) {
        return false;
      }
      pthread_cond_wait(&condition_variable, &mutex);
    }
    *result = elements[head_index];
    head_index = (head_index + 1) % Capacity;
    size--;
    return true;
  }
};

}  // namespace realtime
using namespace realtime;
}  // namespace common

#endif  // CONCURRENT_QUEUE_H
