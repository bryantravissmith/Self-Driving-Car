#include<iostream>


int main()
{
    bool A;
    bool B;
    bool C;
    bool Q;

    for(int i=0;i<=1;++i){
        A = i==1;
        for(int j=0; j<=1; ++j){
            B = j==1;
            for(int k=0; k<=1; ++k){
                C = k==1;
                Q = (A and B and C) or (A and ( (not B) or (not C)));
                std::cout<<A<<"\t"<<B<<"\t"<<C<<"\t";
                std::cout<<(A && B && C)<<"\t\t"<<(!B||!C)<<"\t\t";
                std::cout<<(A&&(!B||!C))<<"\t\t"<<Q<<"\n";
            }
        }
    }

    return 0;
}
