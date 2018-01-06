#include "math.h"
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    p_error=0.0;
    i_error=0.0;
    d_error=0.0;
    is_first=true;
    cnt=0;
    avgerr=0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;
}

void PID::UpdateError(double cte) {
    if (is_first) {
        d_error=0.0;
        is_first=false;
    } else {
        d_error=(cte-p_error);
    }
    p_error=cte;
    i_error+=cte;

    cnt++;
    avgerr+=cte*cte;
    if (fabs(cte) > maxerr) {
        maxerr=fabs(cte);
    }
}

double PID::TotalError() {
    avgerr/=cnt;
    return avgerr;
}

double PID::MaximumError() {
    return maxerr;
}

int PID::NumSteps() {
    return cnt;
}

double PID::computeSteering(double cte, double speed) {
    UpdateError(cte);
    double Kdtmp=Kd + 0.2*speed;
    return -Kp*p_error -Ki*i_error - Kdtmp*d_error;
}

