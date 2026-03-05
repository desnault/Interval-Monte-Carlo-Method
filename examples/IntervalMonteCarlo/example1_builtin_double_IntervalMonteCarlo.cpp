/********************************************************************************************
 * @file    example1_builtin_double_IntervalMonteCarlo.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * 
 * @brief
 * Example demonstrating how to create and use a custom IntervalMonteCarlo estimator
 * for simple double-type samples, including batch processing.
 *
 * @details
 * This example illustrates:
 *   - How to derive a custom class from IntervalMonteCarlo<T> with T = double
 *   - How to implement the `classify()` method for user-defined samples
 *   - How to generate Monte-Carlo samples and process them in multiple batches
 *   - How the estimated probability bound evolves as the total number of processed
 *     samples increases (law of large numbers intuition)
 *
 * Samples are generated using a uniform random distribution on [0,1].
 * The event is evaluated using three-valued logic with the following rules:
 *   - Sample > 0.6  => TRUE
 *   - Sample < 0.4  => FALSE
 *   - Otherwise     => UNKNOWN
 *
 * Notes:
 *   - This example intentionally uses batches (3 x 100 samples) to demonstrate
 *     incremental processing: the estimator keeps memory of previously processed
 *     samples and updates the bound after each batch.
 ********************************************************************************************/

#include "codac.h"

#include "IntervalMonteCarlo.h"

#include <vector>
#include <random>
#include <iostream>
#include <cstdlib>

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
    // Number of batch to generate
    size_t n_batch = 3;
    std::cout<<"\nNumber of batch: "<<n_batch<<std::endl;

    // Number of samples to generate per batch
    size_t n_samples = 100;
    std::cout << "Number of samples per batch: " << n_samples << std::endl << std::endl;

    // Create a random number generator using a non-deterministic seed
    std::random_device rd;
    std::mt19937 gen(rd());

    // Create a uniform distribution generator on [0,1] to generate samples
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // Display info on samples generator
    std::cout<< "Samples will be generated using a uniform distribution on [0,1]" << std::endl << std::endl;

    // Create a vector to save each batch of samples
    std::vector<std::vector<double>> data(n_batch, std::vector<double>(n_samples));

    // Generate the batch(es)
    for (size_t i=0; i<n_batch; i++)
    {
        for (size_t j=0; j<n_samples; j++)
        {
            data[i][j] = uniform(gen);
        }
    }

    // Print classification rules for clarity
    std::cout << "Realization of the event:\n";
    std::cout << "\t- Sample < 0.4 => event is FALSE\n";
    std::cout << "\t- Sample > 0.6 => event is TRUE\n";
    std::cout << "\t- Otherwise (0.4 <= Sample <= 0.6) => event is UNCERTAIN"<< std::endl << std::endl;

    // Create the custom IntervalMonteCarlo estimator
    MyCustomIntervalMonteCarlo my_estimator;

    // Process each batch
    for (size_t i=0; i<n_batch; i++)
    {
        // Update the estimator with current batch
        my_estimator.process_samples(data[i]);

        // Display the info
        std::cout<<"Batch "<<i+1<<"/"<<n_batch<<":"<<std::endl;
        std::cout<<"\tNumber of sample(s) in the batch: "<<n_samples<<std::endl;
        std::cout<<"\tTotal number of sample(s) processed by the estimator: "<<my_estimator.get_number_of_samples()<<std::endl;
        std::cout<<"\tCurrent interval estimate computed by the estimator: "<<my_estimator.get_probability_bound()<<std::endl;
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}