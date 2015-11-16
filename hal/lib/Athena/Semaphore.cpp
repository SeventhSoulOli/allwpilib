#include "HAL/Semaphore.hpp"

#include "Log.hpp"

// set the logging level
TLogLevel semaphoreLogLevel = logDEBUG;

#define SEMAPHORE_LOG(level) \
    if (level > semaphoreLogLevel) ; \
    else Log().Get(level)

MUTEX_ID initializeMutexNormal() { return new std::mutex; }

void deleteMutex(MUTEX_ID sem) { delete sem; }

/**
 * Lock the mutex, blocking until it's available.
 */
void takeMutex(MUTEX_ID mutex) { mutex->lock(); }

/**
 * Attempt to lock the mutex.
 * @return true if succeeded in locking the mutex, false otherwise.
 */
bool tryTakeMutex(MUTEX_ID mutex) { return mutex->try_lock(); }

/**
 * Unlock the mutex.
 * @return 0 for success, -1 for error. If -1, the error will be in errno.
 */
void giveMutex(MUTEX_ID mutex) { mutex->unlock(); }

MULTIWAIT_ID initializeMultiWait() { return new std::condition_variable; }

void deleteMultiWait(MULTIWAIT_ID cond) { delete cond; }

void takeMultiWait(MULTIWAIT_ID cond, MUTEX_ID m) {
  std::unique_lock<std::mutex> lock(*m);
  cond->wait(lock);
}

void giveMultiWait(MULTIWAIT_ID cond) { cond->notify_all(); }

NATIVE_MULTIWAIT_ID getNativeHandle(MULTIWAIT_ID cond) { return cond->native_handle(); }
