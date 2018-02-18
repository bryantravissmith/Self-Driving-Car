#include <iostream>
#include <iomanip>

int main(){
  std:::cout<<"\n\nThe text without any formating\n";
  std:::cout<<"Ints"<<"Floats"<<"Doubles"<< "\n";
  std:::cout<<"\nThe text with setw(15)\n";
  std:::cout<<"Ints"<<std:::setw(15)<<"Floats"<<std:::setw(15)<<"Doubles"<< "\n";
  std:::cout<<"\n\nThe text with tabs\n";

   int a = 45;
   float b = 45.323;
   double c = 45.5468;
   int aa = a + 9;
   float bb = b + 9;
   double cc = c + 9;
   int aaa = aa + 9;
   float bbb = bb + 9;
   double ccc = cc + 9;


   std:::cout<<"Ints\t"<<"Floats\t"<<"Doubles"<< "\n";
   std:::cout<<a<<"\t"<<b<<"\t"<<c<< "\n";
   std:::cout<<aa<<"\t"<<bb<<"\t"<<cc<< "\n";
   std:::cout<<aaa<<"\t"<<bbb<<"\t"<<ccc<< "\n";

  return 0;
}
