#include <iostream>
#include <type_traits>
using namespace std;


struct Person
{
    Person(string&)
    {
        cout << "左值" << endl;
    }

    Person(string&&)
    {
        cout << "右值" << endl;
    }
};


// 问题：
// 1. param 参数会导致对象发生拷贝、移动操作
// 2. param 传递到构造函数，无法区分值的类别(左值、右值)

// 如何避免拷贝和移动？
// 使用引用参数
// 左值引用：就无法接收右值参数
// 右值引用：就无法接收左值参数
// 常量引用：万能引用，既可以引用左值、也可以引用右值
// 原因：由于我们写的是常量引用，这样使得传递的参数增加 const 性
// 希望参数“原封不动”传递到目标函数中

// C++ 中支持泛型（模板技术），函数模板而言，很重要的特性，可以实现类型的自动推导
// T
// T&  只能匹配左值
// T&& 可以了
// const T& 不能写，会给参数增加额外的 const 性
template<class T>
Person make_person(T&& param)
{
    // 疑问：由于写的是 T&& 看起来是一个右值引用，是不是说，无论传递进来的是左值还是右值，都被转换为右值类型？
    // 看看，参数究竟能否正确区分左值和右值
   
    cout << "是否左值:" << is_lvalue_reference<T&&>::value << endl;
    cout << "是否右值:" << is_rvalue_reference<T&&>::value << endl;

    // T&& 通过引折叠能够确定最终的类型：左值引用、右值引用
    // 左值引用：理解为对左值对象的一个别名
    // 右值引用：右值是即将被废弃的对象，右值引用目的就是为了给这些即将废弃的对象续命
    // 此时，无论左值对象、右值对象都变成具名对象（有名字的对象），是一个左值对象了
    // 怎么解决？
    // 将 param 再转换回原来的类型，传递到 Person 构造函数里就可以了
    // return Person((T&&)param);
    // 支持，实现C++参数的完美转发

    // std::forward();
    // return Person(static_cast<T&&>(param));
   
    // 建议大家使用 forward 函数
    return Person(forward<T>(param));
}


void test()
{
    // 1. 传递左值
    string name = "Obama";
    make_person(name);
    
    // 2. 传递右值
    make_person(move(name));
}


int main()
{
    test();
    return 0;
}