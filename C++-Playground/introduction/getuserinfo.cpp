/*Goal: practice std:::cin for strings
**Write a program that prompts two users for their
**name, address, and phone number.
**Print the information to the console in the following format:
**name
**\/t\/t address
**\/t\/tphone number
*/

#include <iostream>
#include <string>

int main(){

    std::string user1Name;
    std::string user2Name;
    std::string user1Address;
    std::string user2Address;
    std::string user1Phone;
    std::string user2Phone;

    std::cout<<"User 1, What is your name? ";
    std::getline(std::cin, user1Name);
    std::cout<<user1Name<<", what is your address? ";
    std::getline(std::cin, user1Address);
    std::cout<<user1Name<<", what is your phone number? ";
    std::getline(std::cin, user1Phone);

    std::cout<<"User 2, What is your name? ";
    std::getline(std::cin, user2Name);
    std::cout<<user2Name<<", what is your address? ";
    std::getline(std::cin, user2Address);
    std::cout<<user2Name<<", what is your phone number? ";
    std::getline(std::cin, user2Phone);

    std::cout<<'\n';
    std::cout<<user1Name<<"\t"<<user1Address<<"\t"<<user1Phone;
    std::cout<<user2Name<<"\t"<<user2Address<<"\t"<<user2Phone;
}
