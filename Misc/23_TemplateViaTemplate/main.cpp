#include <iostream>

class sample
{
    public:
        template<int T>
        void hello()
        {
            std::cout << T << std::endl;
        }
};

template<typename T>
void test()
{
    T s;
#ifdef _WINDOWS
    // or s.template hello<10>();
    s.hello<10>();
#else
    // s.hello<10>(); // gcc says ' error: invalid operands of types ‘<unresolved overloaded function type>’ and ‘int’ to binary ‘operator<’'
    s.template hello<10>();
#endif
}

int main ()
{
    test<sample>();
}