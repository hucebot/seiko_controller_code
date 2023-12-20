#include <stdexcept>
#include <inria_maths/Polynomial.hpp>

namespace inria {
        
Polynomial::Polynomial() :
    _coefs()
{
}
Polynomial::Polynomial(unsigned int degree) :
    _coefs()
{
    _coefs.resize(degree+1, 0.0);
}

const std::vector<double>& Polynomial::getCoefs() const
{
    return _coefs;
}
std::vector<double>& Polynomial::getCoefs()
{
    return _coefs;
}
        
const double& Polynomial::operator()(size_t index) const
{
    return _coefs.at(index);
}
double& Polynomial::operator()(size_t index)
{
    return _coefs.at(index);
}
        
size_t Polynomial::degree() const
{
    return _coefs.size()-1;
}

double Polynomial::pos(double t) const
{
    double tt = 1.0;
    double val = 0.0;
    for (size_t i=0;i<_coefs.size();i++) {
        val += tt*_coefs[i];
        tt *= t;
    }
    return val;
}
double Polynomial::vel(double t) const
{
    double tt = 1.0;
    double val = 0.0;
    for (size_t i=1;i<_coefs.size();i++) {
        val += i*tt*_coefs[i];
        tt *= t;
    }
    return val;
}
double Polynomial::acc(double t) const
{
    double tt = 1.0;
    double val = 0.0;
    for (size_t i=2;i<_coefs.size();i++) {
        val += (i-1)*i*tt*_coefs[i];
        tt *= t;
    }
    return val;
}
double Polynomial::jerk(double t) const
{
    double tt = 1.0;
    double val = 0.0;
    for (size_t i=3;i<_coefs.size();i++) {
        val += (i-2)*(i-1)*i*tt*_coefs[i];
        tt *= t;
    }
    return val;
}

void Polynomial::operator*=(double coef)
{
    for (size_t i=0;i<_coefs.size();i++) {
        _coefs[i] *= coef;
    }
}

void Polynomial::fitCubic(
    double time, 
    double pos1, double vel1,
    double pos2, double vel2)
{
    if (time <= 1e-6) {
        throw std::logic_error(
            "inria::Polynomial:fitCubic: "
            "Invalid time interval: "
            + std::to_string(time));
    }
    double t = time;
    double t2 = t*t;
    double t3 = t2*t;
    getCoefs().resize(4);
    getCoefs()[0] = pos1;
    getCoefs()[1] = vel1;
    getCoefs()[3] = (vel2 - vel1 - 2.0*(pos2-pos1-vel1*t)/t)/t2;
    getCoefs()[2] = (pos2 - pos1 - vel1*t - getCoefs()[3]*t3)/t2;
}
void Polynomial::fitQuintic(
    double time, 
    double pos1, double vel1, double acc1,
    double pos2, double vel2, double acc2)
{
    if (time <= 1e-6) {
        throw std::logic_error(
            "inria::Polynomial:fitQuintic: "
            "Invalid time interval: "
            + std::to_string(time));
    }
    double t = time;
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;
    getCoefs().resize(6);
    getCoefs()[0] = pos1;
    getCoefs()[1] = vel1;
    getCoefs()[2] = acc1/2;
    getCoefs()[3] = -(-acc2*t2+3*acc1*t2+8*vel2*t+12*vel1*t-20*pos2+20*pos1)/(2*t3);
    getCoefs()[4] = (-2*acc2*t2+3*acc1*t2+14*vel2*t+16*vel1*t-30*pos2+30*pos1)/(2*t4);
    getCoefs()[5] = -(-acc2*t2+acc1*t2+6*vel2*t+6*vel1*t-12*pos2+12*pos1)/(2*t5);
}

}

