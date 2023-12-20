#ifndef INRIA_MATHS_POLYNOM_HPP
#define INRIA_MATHS_POLYNOM_HPP

#include <cstdlib>
#include <vector>
#include <iostream>

namespace inria {

/**
 * Polynomial
 *
 * Simple one dimensional 
 * polynomial class for spline 
 * generation
 */
class Polynomial
{
    public:

        /**
         * Default and initial 
         * degree initialization
         *
         * @param degree Initial degree
         * for memory allocation
         */
        Polynomial();
        Polynomial(unsigned int degree);

        /**
         * Access to all coefficients.
         *
         * @return a reference to coefficients container
         * indexed from constant to higher degree.
         */
        const std::vector<double>& getCoefs() const;
        std::vector<double>& getCoefs();

        /**
         * Access to a coefficient.
         *
         * @param index A coefficient index
         * from 0 (constant) to degree()+1.
         * @return a reference to the coefficient.
         */
        const double& operator()(size_t index) const;
        double& operator()(size_t index);

        /**
         * @return the degree of the polynomial.
         * (size_t)-1 mean empty polynomial.
         * There is degree()+1 coefficients.
         */
        size_t degree() const;

        /**
         * Evaluation of the polynomial and 
         * its first, second, third and fourth 
         * derivatives.
         *
         * @param t The evaluation point.
         * @return Polynomial evaluation at t.
         */
        double pos(double t) const;
        double vel(double t) const;
        double acc(double t) const;
        double jerk(double t) const;

        /**
         * Multiply and update all coefficients 
         * with the given constant.
         *
         * @param coef The value to multiply.
         */
        void operator*=(double coef);

        /**
         * Reset internal data
         * and build a cubic or quintic
         * polynomial with given boundary
         * conditions between 0 and time.
         * Throw std::logic_error if given time
         * is zero or negative.
         *
         * @param time Given time of final
         * boundary conditions.
         * @param pos1 Initial position at t=0
         * @param vel1 Initial velocity at t=0
         * @param acc1 Initial acceleration at t=0
         * @param pos2 Final position at t=time
         * @param vel2 Final velocity at t=time
         * @param acc2 Final acceleration at t=time
         */
        void fitCubic(
            double time, 
            double pos1, double vel1,
            double pos2, double vel2);
        void fitQuintic(
            double time, 
            double pos1, double vel1, double acc1,
            double pos2, double vel2, double acc2);

    private:

        /**
         * Polynomial coeficients
         * indexed from constant 
         * to higher degrees
         */
        std::vector<double> _coefs;
};

}

#endif

