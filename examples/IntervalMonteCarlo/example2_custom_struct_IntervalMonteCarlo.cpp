/********************************************************************************************
 * @file    example2_custom_struct_IntervalMonteCarlo.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * 
 * @brief   Example of the Interval Monte-Carlo Method (IMCM) using custom mission samples.
 *
 * @details
 * This example illustrates how to use the IntervalMonteCarlo class with a user-defined
 * structured sample type representing an AUV identification mission.
 *
 * Each Monte Carlo sample encodes:
 *  - a robot trajectory (serialized TubeVector loaded from disk),
 *  - an uncertain object position (2D IntervalVector box),
 *  - a sensor detection range.
 *
 * The event to evaluate is: "the object is detected during the mission".
 * Because the object position is uncertain (set-based), the event realization follows a
 * three-valued logic:
 *  - TRUE    : detection is guaranteed (box fully covered),
 *  - FALSE   : detection is impossible (box not covered at all),
 *  - UNKNOWN : detection is uncertain (partial coverage).
 *
 * Practical workflow demonstrated in this script:
 *  1) Load a small dataset of TubeVector trajectories (*.tubevector) from the repo data folder.
 *  2) Convert each trajectory into an AUV_Mission_Sample.
 *  3) Update the estimator sample-by-sample (process_sample) and display the local evaluation.
 *  4) Read the final empirical probability bound [P_E] after all samples have been processed.
 *
 * Note:
 *  - For clarity, the local evaluation is obtained by calling evaluate_sample() in addition to
 *    process_sample(). This re-evaluates the sample and is intended for demonstration purposes.
 ********************************************************************************************/

#include "codac.h"

#include "SepDynDiskProj.h"
#include "BoxInclusionClassifier.h"
#include "IntervalMonteCarlo.h"

#include <vector>
#include <iostream>
#include <string>
#include <filesystem>
#include <cstdlib>



/*==================================================================================
 * DATASET PARAMETERS
 *==================================================================================
 *
 * This example loads pre-generated trajectories from disk (no simulation here).
 * The time bounds below are used for the sensor coverage projection.
 *==================================================================================*/

// Initial and final time of the trajectory
const double t0 = 0.55;
const double tf = 12;


/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

// Time period over which to project the sensor coverage
// Here we consider the whole trajectory
const Interval t_proj(t0, tf);

// Temporal resolution of the projection (smaller = finer resolution)
const double eps_proj = 0.01;


/*==================================================================================
 * MISSION PARAMETERS
 *==================================================================================*/

// Box containing the object to detect
const IntervalVector box_to_cover = {{-0.5, 0.5},{-0.5,0.5}};
    
// Detection range of the robot
const double detection_range = 1.;


/*==================================================================================
 * CUSTOM INTERVAL MONTE CARLO
 *==================================================================================*/

// AUV mission sample definition.
//
// Each sample represents a single realization of an AUV mission and includes:
//   - a robot trajectory (loaded from disk),
//   - an uncertain object position modeled as a 2D interval box,
//   - the detection range of the onboard sensor.
//
// The objective is to determine whether the object is detected at least once
// along the trajectory.
struct AUV_Mission_Sample 
{
    TubeVector tube_trajectory;
    IntervalVector object_to_detect;
    double detection_range;
};

/*
 * @class MyCustomIntervalMonteCarlo
 *
 * @brief Interval Monte Carlo estimator specialized for AUV mission samples
 *
 * @details
 * This class provides a concrete implementation of the IntervalMonteCarlo
 * framework for autonomous underwater vehicle (AUV) missions.
 *
 * Each Monte Carlo sample encodes a complete mission scenario, including
 * the robot trajectory, sensor characteristics, and an uncertain object position.
 * The classify() method evaluates whether the object is detected along the
 * trajectory using a three-valued logic based on set inclusion.
 *
 * This example illustrates how IntervalMonteCarlo can be adapted to complex
 * application-specific data structures while preserving the same estimation
 * principles.
 *
 * Internally, the covered area is represented by a CODAC separator (SepDynDiskProj),
 * and the event is evaluated by classifying the object box using BoxInclusionClassifier.
 * 
 * @note
 * Only the sample type and the classify() method are application-dependent.
 * The Monte Carlo estimation logic is fully handled by the base class.
 */

class MyCustomIntervalMonteCarlo : public IntervalMonteCarlo<AUV_Mission_Sample>
{
    public:
        THREE_VALUED_LOGIC classify(const AUV_Mission_Sample& sample) const override
        {
            // Create a separator representing the sensor coverage along the trajectory
            SepDynDiskProj my_separator(sample.tube_trajectory, sample.detection_range, t_proj, eps_proj, true);

            // Create a box inclusion classifier to evaluate object coverage
            BoxInclusionClassifier my_classifier(my_separator);

            // Classify the object position with respect to the covered area
            Interval classification_result = my_classifier.classify(sample.object_to_detect, sample.object_to_detect.max_diam()/10.);

            // Return the corresponding three-valued logic value
            if (classification_result == Interval(1))
            {
                return THREE_VALUED_LOGIC::TRUE;
            }
            
            if (classification_result == Interval(0))
            {
                return THREE_VALUED_LOGIC::FALSE;
            }

            return THREE_VALUED_LOGIC::UNKNOWN;
        };
};



/*==================================================================================
 * MAIN
 *==================================================================================*/

int main()
{

    // ----------------------------------------------------
    // MONTE CARLO SAMPLE LOAD
    // ----------------------------------------------------

    // Vector containing all Monte Carlo samples.
    // Each sample represents one possible realization of an AUV mission.
    std::vector<AUV_Mission_Sample> samples = {};

    // Get path to the folder containing the trajectories
    // INTERVAL_MC_DATA_DIR is expected to point to the repo "data" folder (set by CMake).
    // This example loads trajectories from: ${INTERVAL_MC_DATA_DIR}/examples
    std::string trajectory_directory = INTERVAL_MC_DATA_DIR;
    trajectory_directory = trajectory_directory + "/examples";

    // Load each trajectory and convert it into an AUV_Mission_Sample
    for (const auto& entry : std::filesystem::directory_iterator(trajectory_directory))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".tubevector")
        {
            TubeVector my_tube(entry.path().string());
            AUV_Mission_Sample sample = {my_tube, box_to_cover, detection_range};
            samples.push_back(sample);
        }
    }

    std::cout<< "\nNumber of trajectory(ies) loaded: "<<samples.size()<<std::endl<<std::endl;

    
    // ----------------------------------------------------
    // INTERVAL MONTE CARLO ESTIMATION
    // ----------------------------------------------------

    // Create the custom three-valued logic Monte Carlo estimator
    MyCustomIntervalMonteCarlo my_estimator;

    // Process all samples and update the empirical probability bound
    // my_estimator.process_samples(samples);

    // OR

    // Process each sample individulay to verify inclusion evaluation
    for (size_t i=0; i<samples.size(); i++)
    {
        // Update estimator with sample
        my_estimator.process_sample(samples[i]);

        // Get evaluation of the classify() method for the sample (only for display purpose)
        Interval sample_evaluation = my_estimator.evaluate_sample(samples[i]);
        // Display evaluation for the current trajectory
        std::cout<<"Trajectory "<<i+1<<"/"<<samples.size()<<":"<<std::endl;
        std::cout<<"\tLocal evaluation: "<<sample_evaluation<<std::endl;
    }

    // Get the interval estimate of the success probability
    Interval estimated_probability = my_estimator.get_probability_bound();
    std::cout<< "\nThe estimated interval probability is: "<< estimated_probability << std::endl<< std::endl;

    return EXIT_SUCCESS;
}