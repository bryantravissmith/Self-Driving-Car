#include "Planner.h"

using namespace std;


Planner::Planner(){ }

/* Destructor */
Planner::~Planner(){ }

/* Initialize */
void Planner::Init(Executor& e){
  this->exector = e;
}

/* Update the PID error variables given cross track error */
void Planner::Update(){

}

double Planner::Cost(){
    return 0.0;
}
