/*Goal: practice if-else statements in C++
**Write a program to select the best pet.
*/

#include<iostream>
#include <string>
#include <sstream>

int main()
{
   char skin, location;
   std::cout<<"Would you like an animal with fur(f), feathers(t), or scales(s)?";
   std::cin>>skin;
   std::cout<<skin<<"\n";

   if(skin=='f'){
       std::cout<<"Get a dog"<<"\n";
   }
    else if(skin=='t'){
        std::cout<<"Get a bird"<<"\n";
    } else {
        std::cout<<"Would you like an animal that lives in water(w), land(l), or both(b)?";
        std::cin>>location;
        std::cout<<location<<"\n";

        if(location=='w'){
            std::cout<<"Get a fish"<<"\n";
        } else if (location=='l'){
            std::cout<<"Get a gecko"<<"\n";
        } else {
            std::cout<<"Get a frog"<<"\n";
        }

    }
   return 0;
}
