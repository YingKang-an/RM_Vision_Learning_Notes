#include <iostream>
#include <thread>
#include <mutex>

// lock_guard 是一个模板类， 位于 <mutex> 头文件中 。 
// 它符合 RAII风格，它主要用于管理 mutex 的生命周期，确保 mutex 在锁定的作用域内被正确地上锁和解锁。
// 它主要解决了手动管理 mutex 锁定和解锁时可能出现的问题，如：忘记解锁、异常情况下未解锁等问题。 
// 你可以简单把这个东西，看成是对mutex的一种管理封装，就是说用原生的mutex，有些时候不顺手，有瑕疵。
// 使用lock_guard 比较省心，它来更好地管理你的mutex。
