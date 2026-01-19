/*
 * @file IntervalMonteCarlo.h
 * @author Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date 2025
 *
 * @brief Implementation of a three-valued logic Monte Carlo estimator.
 *
 * @details
 * This class provides an adaptation of the classical Monte Carlo estimator
 * to handle **uncertain realizations** using three-valued logic: TRUE, FALSE,
 * and UNKNOWN. Each sample is classified as fully satisfying an event (TRUE),
 * fully not satisfying (FALSE), or uncertain/partially satisfying an event (UNKNOWN).  
 *
 * Compared to the standard binary Monte Carlo estimator, which computes a single
 * estimate of the probability of an event, the three-valued logic Monte Carlo
 * estimator computes a **probability bound** represented as an Interval.  
 * This interval is not a standard stochastic confidence interval, but an
 * **empirical bound guaranteed in the limit by the law of large numbers**: as 
 * the number of samples N approaches infinity, the true probability of the event 
 * is guaranteed to lie within the interval.
 *
 * The class is **templated** over the type of sample `T`, allowing users to define
 * their own sample structure. The key customization point is the `classify()` method,
 * which must be defined in a derived class to classify samples according to
 * the user’s problem.  
 *
 * The class is designed for **batch processing** of samples. Users can provide
 * all samples at once or process them in multiple smaller batches. For instance,
 * to estimate a probability bound over 10,000 samples, one can:
 *   - Feed all 10,000 samples at once using `process_samples()`, or
 *   - Feed 1,000 samples at a time in 10 consecutive calls to `process_samples()`.
 * This is especially useful when the samples are large in memory, or the
 * computer has limited resources, allowing users to sequentially process chunks
 * of data without exceeding memory limits.
 *
 * Typical usage:
 *   1. Subclass `IntervalMonteCarlo<T>` and define `THREE_VALUED_LOGIC classify(const T&)`.
 *   2. Create an instance of your subclass.
 *   3. Call `process_sample` or `process_samples` to update the estimator.
 *   4. Retrieve the interval probability bound with `get_probability_bound()`.
 *
 * Example applications include robot coverage estimation, stochastic system reliability,
 * or any domain requiring interval-based Monte Carlo evaluation.
 */

#ifndef __INTERVAL_MONTE_CARLO_H__
#define __INTERVAL_MONTE_CARLO_H__

#include "codac.h"

#include <vector>
#include <fstream>
#include <iomanip>

// Note: using namespace codac is intentionally used here for readability,
// as this class is tightly coupled with CODAC interval types.
using namespace codac;


// Three-valued logic used by IntervalMonteCarlo to represent the outcome
// of a sample evaluation: TRUE (event occurs), FALSE (event does not occur),
// or UNKNOWN (uncertain outcome, partially true/false).
enum class THREE_VALUED_LOGIC {TRUE, FALSE, UNKNOWN};


/*
 * @class IntervalMonteCarlo
 *
 * @brief Three-valued logic Monte Carlo estimator.
 *
 * @details
 * This class implements a Monte Carlo probability estimator where each sample
 * can be TRUE (event occurs), FALSE (event does not occur), or UNKNOWN (uncertain).  
 *
 * Interval bounds are accumulated to reflect uncertainty:
 *   - TRUE  → [1]
 *   - FALSE → [0]
 *   - UNKNOWN → [0,1]
 *
 * The estimator maintains:
 *   - `_n_samples`: number of samples processed
 *   - `_sum`: interval sum of all sample evaluations
 *
 * Key features:
 *   - `classify(const T&)` must be implemented by the user to define how a sample
 *     is evaluated according to the problem.
 *   - `evaluate_sample(const T&)` provides the interval corresponding to a single
 *     sample (helpful for debugging).
 *   - `process_sample` and `process_samples` update the estimator with new data.
 *   - `process_samples_and_export` can save the evolution of the probability bound
 *     to a text file for convergence analysis.
 *
 * The class is general and can be applied to robotics, reliability studies,
 * stochastic systems, or any Monte Carlo application requiring interval estimates.
 */
template<typename T>
class IntervalMonteCarlo
{
    public:

        /*
         * @brief Constructor
         *
         * @details
         * Default constructor. Initializes the estimator counters and sum to zero.
         *
         * @note
         * For a fully functional estimator, the user should subclass and implement
         * the `classify()` method.
         */
        IntervalMonteCarlo() = default;
        
        /*
         * @brief Destructor
         *
         * @details
         * Default destructor. Cleans up the estimator object.
         *
         * @note
         * Nothing special is needed because no dynamic memory is allocated internally.
         */
        virtual ~IntervalMonteCarlo() = default;

        /*
         * @brief Classify a sample according to three-valued logic.
         *
         * @details
         * This pure virtual method must be implemented by the user in a derived class.
         * Each sample should be classified as:
         *   - THREE_VALUE_LOGIC::TRUE   → sample fully satisfies the event
         *   - THREE_VALUE_LOGIC::FALSE  → sample does not satisfy the event
         *   - THREE_VALUE_LOGIC::UNKNOWN → sample outcome is uncertain
         *
         * The returned value is later converted into an interval for the probability
         * estimate ([1], [0], or [0,1]).
         *
         * @param sample: The sample to classify
         *
         * @note
         * This is the only method that depends on the user. Everything else
         * in the class is independent of the specific type of sample or application.
         */
        virtual THREE_VALUED_LOGIC classify(const T& sample) const = 0;

        /*
         * @brief Reset the estimator
         *
         * @details
         * Sets `_n_samples` and `_sum` to zero.
         * The estimator can then be reused for a new set of samples.
         */
        void reset()
        {
            _n_samples = 0;
            _sum = Interval(0);
        }

        /*
         * @brief Get the number of samples processed
         *
         * @details
         * Returns `_n_samples`.
         */
        std::size_t get_number_of_samples() const
        {
            return _n_samples;
        }
        
        /*
         * @brief Get the current interval probability bound
         *
         * @details
         * Returns the interval probability bound calculated as:
         *     _sum / _n_samples
         *
         * If no samples have been processed, returns [0,1].
         *
         * @return Interval representing the probability bound
         */
        Interval get_probability_bound() const
        {
            if (_n_samples == 0)
            {
                return Interval(0,1);
            }
            else
            {
                return _sum*(1./static_cast<double>(_n_samples));
            }
        }

        /*
         * @brief Evaluate a single sample and return its interval realization.
         *
         * @details
         * Converts a sample into a three-valued logic interval:
         *   - TRUE   → Interval(1)
         *   - FALSE  → Interval(0)
         *   - UNKNOWN → Interval(0,1)
         *
         * This method **does not update the estimator**. It is intended as a
         * debug utility to check the behavior of the user-defined `classify` method.
         *
         * @param sample: The sample to evaluate
         *
         * @return Interval representing the sample's contribution to the probability
         *
         * @note
         * Useful for unit tests or inspecting individual sample outcomes.
         */
        Interval evaluate_sample(const T& sample) const
        {
            switch (classify(sample))
            {
                case THREE_VALUED_LOGIC::TRUE:
                    return Interval(1);
                case THREE_VALUED_LOGIC::FALSE:
                    return Interval(0);
                case THREE_VALUED_LOGIC::UNKNOWN:
                    return Interval(0,1);
                default:
                    throw std::runtime_error("[ERROR] IntervalMonteCarlo::evaluate_sample(): invalid THREE_VALUED_LOGIC");
            }
        }

        /*
         * @brief Update the estimator with a single sample.
         *
         * @details
         * Evaluates the sample using `evaluate_sample` and adds its interval
         * to the accumulated sum. Increments the sample counter.
         *
         * This is the core method of the estimator. Repeated calls with multiple
         * samples allow the probability bound to converge.
         *
         * @param sample: The sample to process
         *
         * @note
         * For multiple samples, prefer `process_samples` for convenience.
         */
        void process_sample(const T& sample) 
        {
            _sum +=  evaluate_sample(sample);
            ++_n_samples;
        }
        
        /*
         * @brief Update the estimator with a vector of samples.
         *
         * @details
         * Iterates through the input vector, evaluates each sample,
         * and updates the estimator by accumulating interval sums and sample count.
         *
         * Throws an exception if the vector is empty.
         *
         * @param samples: vector of samples to process
         *
         * @note
         * Internally calls `evaluate_sample` for each sample.
         */
        void process_samples(const std::vector<T>& samples)
        {
            // Verify that the samples vector is not empty
            if (samples.empty())
            {
                throw std::range_error("[ERROR] IntervalMonteCarlo::process_samples(): Samples vector is empty");
            }

            // Process each sample in the input vector
            for (const auto& s: samples)
            {   
                _sum +=  evaluate_sample(s);
                ++_n_samples;
            }
        }
        
        /*
         * @brief Update the estimator with samples and export evolution to a text file.
         *
         * @details
         * Similar to `process_samples`, but writes the evolution of the interval probability
         * bound after each sample to a text file. Useful for studying convergence
         * of the estimator.
         *
         * The file format:
         *   n [lower_bound, upper_bound]
         *
         * Throws an exception if the input vector is empty or the file cannot be opened.
         *
         * @param samples: vector of samples to process
         * @param filename: path to the output file
         *
         * @note
         * This method is intended for monitoring or debugging Monte Carlo runs.
         */
        void process_samples_and_export(const std::vector<T>& samples, const std::string& filename)
        {
            // Verify that the samples vector is not empty
            if (samples.empty())
            {
                throw std::range_error("[ERROR] IntervalMonteCarlo::process_samples_and_export(): Samples vector is empty");
            }

            // Open file in "append" mode
            std::ofstream file(filename, std::ios::app);
            if (!file.is_open())
            {
                // Throws an exception if the file cannot be opened
                throw std::runtime_error("[ERROR] IntervalMonteCarlo::process_samples_and_export(): Failed to open file " + filename);
            }

            // Process each sample
            for (const auto& s : samples)
            {
                // Update estimator
                _sum +=  evaluate_sample(s);
                ++_n_samples;

                // Update text file
                Interval probability_bound = get_probability_bound();
                file << _n_samples << " [" << probability_bound.lb() << "," << probability_bound.ub() << "]\n";
            }

            // Close file properly
            file.close();
        }
    
    protected:

        // Total number of samples processed by the estimator.
        // Used to compute the average and normalize the probability bound.
        std::size_t _n_samples = 0;
        
        // Cumulative sum of the interval evaluations of all processed samples.
        // Each sample contributes [0], [1], or [0,1] depending on its classification.
        // Used to compute the current probability bound as (_sum / _n_samples).
        Interval _sum = Interval(0);
}; 

# endif // __INTERVAL_MONTE_CARLO_H__