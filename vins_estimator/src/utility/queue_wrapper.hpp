#pragma once

#include <utility/moodycamel/readwrite/readerwriterqueue.h>
#include <queue>
template <typename T>
class RW_Queue {
  private:
	moodycamel::ReaderWriterQueue<T> queue_;

  public:
	RW_Queue(int min_size) : queue_(moodycamel::ReaderWriterQueue<T>(min_size)) {
	}
	RW_Queue() : queue_(moodycamel::ReaderWriterQueue<T>()) {
	}
	~RW_Queue() = default;

	void   push(const T &value);
	void   push(T &&value);
	void   pop();
	bool   try_pop();
	bool   try_pop(T &value);
	T	  *front();
	bool   empty();
	size_t size();
	void   clear();
};

template <typename T>
void RW_Queue<T>::push(const T &value) {
	queue_.enqueue(value);
}
template <typename T>
void RW_Queue<T>::push(T &&value) {
	queue_.enqueue(std::forward<T>(value));
}
template <typename T>
void RW_Queue<T>::pop() {
	T temp;
	queue_.try_dequeue(temp);
}
template <typename T>
bool RW_Queue<T>::try_pop() {
	T temp;
	return queue_.try_dequeue(temp);
}
template <typename T>
bool RW_Queue<T>::try_pop(T &value) {
	return queue_.try_dequeue(value);
}
template <typename T>
T *RW_Queue<T>::front() {
	return queue_.peek();
}
template <typename T>
bool RW_Queue<T>::empty() {
	return queue_.size_approx() == 0;
}
template <typename T>
size_t RW_Queue<T>::size() {
	return queue_.size_approx();
}
template <typename T>
void RW_Queue<T>::clear() {
	T temp;
	while (queue_.try_dequeue(temp))
		;
}


template <typename T>
class Mutex_Queue {
  private:
	std::queue<T> q;
	std::mutex	  mutex_;

  public:
	Mutex_Queue()  = default;
	~Mutex_Queue() = default;

	Mutex_Queue(const Mutex_Queue &other) {
		std::lock_guard<std::mutex> lock(mutex_);
		q = other.q;
	}; // 拷贝构造
	Mutex_Queue &operator=(const Mutex_Queue &other) {
		std::lock_guard<std::mutex> lock(mutex_);
		q = other.q;
		return *this;
	}; // 拷贝赋值
	Mutex_Queue(Mutex_Queue &&other) {
		std::lock_guard<std::mutex> lock(mutex_);
		q = std::move(other.q);
	}; // 移动构造
	Mutex_Queue &operator=(Mutex_Queue &&other) {
		std::lock_guard<std::mutex> lock(mutex_);
		q = std::move(other.q);
		return *this;
	}; // 移动赋值

	void   push(const T &value);
	void   push(T &&value);
	void   pop();
	T	  &front();
	T	  &back();
	bool   empty();
	size_t size();
	void   clear();
};

template <typename T>
void Mutex_Queue<T>::push(const T &value) {
	std::lock_guard<std::mutex> lock(mutex_);
	q.push(value);
}
template <typename T>
void Mutex_Queue<T>::push(T &&value) {
	std::lock_guard<std::mutex> lock(mutex_);
	q.push(std::forward<T>(value)); // 完美转发，注意forward也是模板函数
}

template <typename T>
void Mutex_Queue<T>::pop() {
	std::lock_guard<std::mutex> lock(mutex_);
	q.pop();
}
template <typename T>
T &Mutex_Queue<T>::front() {
	std::lock_guard<std::mutex> lock(mutex_);
	return q.front();
}
template <typename T>
T &Mutex_Queue<T>::back() {
	std::lock_guard<std::mutex> lock(mutex_);
	return q.back();
}
template <typename T>
bool Mutex_Queue<T>::empty() {
	return q.empty();
}
template <typename T>
size_t Mutex_Queue<T>::size() {
	return q.size();
}
template <typename T>
void Mutex_Queue<T>::clear() {
	std::lock_guard<std::mutex> lock(mutex_);
	q.clear();
}
