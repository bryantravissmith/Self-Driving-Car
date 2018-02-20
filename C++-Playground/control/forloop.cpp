#include <iostream>

int main()
{
    float num;
    float sum;

    for(int i=1; i<= 5;i++)
    {
        std::cout<<"Enter a number ("<<i<<"/5)\n";
        std::cin>>num;
        sum += num;
    }
    std::cout<< "Sum: " << sum << ", Avg: " << sum/5;
    return 0;
}
