#include <iostream>

template<typename T>
class sample
{
    public:
        void hello(T value)
        {
            std::cout << value << std::endl;
        }
};

template<typename T>
void test(T value)
{
    sample<T> s;
    s.hello(value);
}

int main ()
{
    test(10);
}