#include <iostream>
 
#ifdef __cplusplus
extern "C"{
#endif
 
int add(int a, int b);
 
#ifdef __cplusplus
}
#endif

int main ()
{
    auto ret = add(10, 2);
    std::cout << "10 + 2 = " << ret << std::endl;
}