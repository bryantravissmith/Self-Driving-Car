#include <iostream>
#include <string>
#include <sstream>

int main(){
  std::string stringLength;
  std::string stringWidth;
  float length = 0;
  float width = 0;
  float area = 0;

  std::cout << "Enter length of room: ";
  std::getline (std::cin,stringLength);
  std::stringstream(stringLength) >> length;


  std::cout << "Enter width of room: ";
  std::getline (std::cin,stringWidth);
  std::stringstream(stringWidth) >> width;

  area = length * width;

  std::cout<<"Area of room: "<<area<<"\n";
  return 0;
}
