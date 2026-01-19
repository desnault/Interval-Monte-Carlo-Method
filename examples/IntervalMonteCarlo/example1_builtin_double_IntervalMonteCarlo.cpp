/********************************************************************************************
 * @file    example1_builtin_double_IntervalMonteCarlo.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * 
 * @brief   
 * Example demonstrating how to create and use a custom IntervalMonteCarlo estimator
 * for simple double-type samples.
 *
 * @details
 * This example illustrates:
 *   - How to derive a custom class from IntervalMonteCarlo<T>
 *   - How to implement the `classify()` method for the user-defined samples
 *   - How to generate samples, process them with the estimator, and obtain
 *     the estimated interval probability bound
 *   - How to interpret three-valued logic outcomes for simple numeric rules
 * 
 * The example uses a uniform random distribution to generate double samples
 * in [0,1], with classification rules:
 *   - Sample > 0.6 => TRUE
 *   - Sample < 0.4 => FALSE
 *   - Otherwise => UNKNOWN
 ********************************************************************************************/

#include "codac.h"

#include "IntervalMonteCarlo.h"

#include <vector>
#include <random>
#include <iostream>

/*
 * @class MyCustomIntervalMonteCarlo
 *
 * @brief Custom IntervalMonteCarlo estimator for double samples
 *
 * @details
 * This class shows how to create a simple implementation of the IntervalMonteCarlo
 * template class for built-in numeric types (double in this case).
 *
 * The user must implement the `classify()` method, which maps a sample to the
 * three-valued logic (TRUE, FALSE, UNKNOWN). This method drives the computation
 * of the probability bound in the parent IntervalMonteCarlo class.
 *
 * In this example:
 *   - A sample greater than 0.6 is considered a TRUE event
 *   - A sample lower than 0.4 is considered a FALSE event
 *   - A sample in [0.4, 0.6] is considered UNKNOWN
 *
 * @note
 * This demonstrates the generality of the IntervalMonteCarlo class: only the sample
 * type and the classify method need to be adapted for other use cases.
 */
class MyCustomIntervalMonteCarlo : public IntervalMonteCarlo<double>
{
    public:
        THREE_VALUED_LOGIC classify(const double& sample) const override
        {
            if(sample>0.6)
            {
                return THREE_VALUED_LOGIC::TRUE;
            }
            if(sample<0.4)
            {
                return THREE_VALUED_LOGIC::FALSE;
            }
            return THREE_VALUED_LOGIC::UNKNOWN;
        };
};


int main()
{
    // Create a random number generator using a non-deterministic seed
    std::random_device rd;
    std::mt19937 gen(rd()); 

    // Uniform distribution in [0,1] to generate test samples
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // Number of samples to generate
    size_t n_samples = 50;
    std::cout << "\nNumber of samples: " << n_samples << "\n";

    // Generate the samples
    std::vector<double> samples;
    samples.reserve(n_samples); // Optimize memory allocation
    for (size_t i = 0; i < n_samples; i++)
    {
       samples.push_back(uniform(gen));
    }

    // Display generated samples
    std::cout << "\nList of samples: {";
    for (size_t i = 0; i < samples.size(); i++)
    {
        std::cout << samples[i];
        if (i != samples.size() - 1) std::cout << ", ";
    }
    std::cout << "}\n\n";

    // Print classification rules for clarity
    std::cout << "Realization of the event:\n";
    std::cout << "\t- Sample < 0.4 => event is FALSE\n";
    std::cout << "\t- Sample > 0.6 => event is TRUE\n";
    std::cout << "\t- Otherwise (0.4 <= Sample <= 0.6) => event is UNCERTAIN\n\n";

    // Create the custom IntervalMonteCarlo estimator
    MyCustomIntervalMonteCarlo my_estimator;

    // Process all samples to update the probability bound
    my_estimator.process_samples(samples);

    // Retrieve the estimated probability interval
    Interval estimation = my_estimator.get_probability_bound();

    // Display the result
    std::cout << "Estimated probability interval: " << estimation << "\n\n";

    return EXIT_SUCCESS;
}