#ifndef REALTIMEIPC_H
#define REALTIMEIPC_H

#include <cstdlib>
#include <cstring>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

// remove if kinetic is no longer used
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <ros/common.h>
THIRD_PARTY_HEADERS_END

namespace common {
namespace realtime {

/**
 * @brief The RealtimeIPC class provides a mechanism for real-time inter-process
 * communication.
 * Internally, it uses xsi shared memory and locking primitives.
 * The communication methods are allocation free.
 *
 * To work correctly the type T must be trivially copyable, which basically
 * means, that it can be safely copied by memcpy().
 * To work correctly the type T must be a standard layout type, which basically
 * means, that is similar to a c-style struct (has no virtual function or base
 * class).
 */
template <class T>
class RealtimeIPC {
#if ROS_VERSION_MINIMUM(1, 13, 1)
  // in kinectic this does not work because of an oddity in ros::Time, which is
  // fixed in melodic.
  static_assert(std::is_trivially_copyable<T>::value, "");
  static_assert(std::is_standard_layout<T>::value, "");
#endif

 public:
  RealtimeIPC(int channel_id);
  ~RealtimeIPC();
  RealtimeIPC(RealtimeIPC&&) = default;

  const T* get();
  void write(const T&);

 private:
  void lock();
  void unlock();

  int shared_memory_id_;
  int semaphore_id_;
  void* shared_memory_;
  void* private_memory_;
};


// Definitions come here (needed for template instantiations)

template <class T>
RealtimeIPC<T>::RealtimeIPC(int channel_id) {
  shared_memory_id_ = shmget(channel_id, sizeof(T), IPC_CREAT | 0666);
  shared_memory_ = shmat(shared_memory_id_, nullptr, 0);
  semaphore_id_ = semget(channel_id, 1, IPC_CREAT | 0666);
  unsigned short sem_init_val = 1;
  semctl(semaphore_id_, 0, SETALL, &sem_init_val);
  private_memory_ = malloc(sizeof(T));
  memset(private_memory_, 0, sizeof(T));
}

template <class T>
RealtimeIPC<T>::~RealtimeIPC() {
  shmdt(shared_memory_);
  free(private_memory_);
}

template <class T>
const T* RealtimeIPC<T>::get() {
  lock();
  memcpy(private_memory_, shared_memory_, sizeof(T));
  unlock();
  return static_cast<T*>(private_memory_);
}

template <class T>
void RealtimeIPC<T>::write(const T& message) {
  lock();
  memcpy(shared_memory_, &message, sizeof(T));
  unlock();
}

template <class T>
void RealtimeIPC<T>::lock() {
  //! @TODO the locking mechanism should really use priority-inheritance mutexes
  //! instead of System V semaphores. However, kernel-managed semaphores don't
  // require
  //! us to introduce further structure in the shared memory, so we'll be using
  // them for
  //! simplicity. As long as all communicating processes have the same realtime
  // priority,
  //! we should be fine. Otherwise this leads to a prime example of priority
  // inversion.
  //! In the long term, this should be changed.
  //! - marek
  struct sembuf semaphore_operation;
  semaphore_operation.sem_num = 0;  // Use first semaphore
  semaphore_operation.sem_op = -1;  // decrement semaphore value
  semaphore_operation.sem_flg = 0;

  if (semop(semaphore_id_, &semaphore_operation, 1) == -1) {
    exit(EXIT_FAILURE);
  }
}

template <class T>
void RealtimeIPC<T>::unlock() {
  struct sembuf semaphore_operation;
  semaphore_operation.sem_num = 0;  // Use first semaphore
  semaphore_operation.sem_op = 1;   // increment semaphore value
  semaphore_operation.sem_flg = 0;

  if (semop(semaphore_id_, &semaphore_operation, 1) == -1) {
    exit(EXIT_FAILURE);
  }
}

}  // namespace realtime
using namespace realtime;
}  // namespace common

#endif  // REALTIMEIPC_H
