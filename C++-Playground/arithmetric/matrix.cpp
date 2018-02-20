/*Goal: practice using multidimensional arrays.
**Write a program that will accept values for a 4x4 array
**and a vector of size 4.
**Use the dot product to multiply the array by the vector.
**Print the resulting vector.
*/

#include<iostream>

int main()
{
    //TODO: multiply a 4x4 array with vector of size 4.
    //Print the resultant product vector
    int matrix[4][4];
    int vector[4];
    int result[4];

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            std::cin>>matrix[i][j];
        }
    }

    for(int j = 0; j < 4; j++){
        std::cin>>vector[j];
    }


    for(int i = 0; i < 4; i++){
        int prod = 0;
        for(int j = 0; j < 4; j++){
            prod += matrix[i][j] * vector[j];
        }
        result[i] = prod;
    }

    for(int i = 0; i < 4; i++){
        std::cout<<result[i]<<" ";
    }
    return 0;
}
