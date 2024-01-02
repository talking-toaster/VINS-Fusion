# 无锁队列库

来自 `https://github.com/cameron314/readerwriterqueue` 和 `https://github.com/cameron314/concurrentqueue`

## 单读单写队列

###

## 多读多写队列

非阻塞,使用头文件 `readerwriterqueue.h`

```cpp
using namespace moodycamel;

ReaderWriterQueue<int> q(100);       // reserve 100,可自动扩展

q.enqueue(17);                       // Will allocate memory if the queue is full
bool succeeded = q.try_enqueue(18);  // Will only succeed if the queue has an empty slot (never allocates)
assert(succeeded);

int number;
succeeded = q.try_dequeue(number);  // Returns false if the queue was empty

assert(succeeded && number == 17);

// You can also peek at the front item of the queue (consumer only)
int* front = q.peek();
assert(*front == 18);
succeeded = q.try_dequeue(number);
assert(succeeded && number == 18);
front = q.peek();
assert(front == nullptr);           // Returns nullptr if the queue was empty
assert(q.size_approx() == 0);
```

```cpp
BlockingReaderWriterQueue<int> q;

std::thread reader([&]() {
    int item;
#if 1
    for (int i = 0; i != 100; ++i) {
        // Fully-blocking:
        q.wait_dequeue(item);
    }
#else
    for (int i = 0; i != 100; ) {
        // Blocking with timeout
        if (q.wait_dequeue_timed(item, std::chrono::milliseconds(5)))
            ++i;
    }
#endif
});
std::thread writer([&]() {
    for (int i = 0; i != 100; ++i) {
        q.enqueue(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
});
writer.join();
reader.join();

assert(q.size_approx() == 0);
```
