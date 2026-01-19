/********************************************************************************************
 * @file    example2_custom_struct_IntervalMonteCarlo.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * 
 * @brief   Example of a three-valued logic Monte Carlo estimator using custom structured samples
 *
 * @details
 * This example illustrates how to use the IntervalMonteCarlo class with user-defined
 * structured samples instead of built-in scalar types.
 *
 * Each Monte Carlo sample represents an AUV mission scenario, including:
 *  - a stochastic robot trajectory (modeled as a TubeVector),
 *  - an uncertain object position (modeled as a 2D interval box),
 *  - a sensor detection range.
 *
 * Because the object position is uncertain and represented as a set, the detection
 * outcome cannot always be classified using binary logic. Instead, a three-valued
 * logic is used:
 *  - TRUE    : the object is guaranteed to be detected,
 *  - FALSE   : the object is guaranteed not to be detected,
 *  - UNKNOWN : the detection outcome is uncertain (partial coverage).
 *
 * This example demonstrates how the three-valued logic Monte Carlo estimator
 * can be applied to robotics problems involving stochastic behaviors and
 * set-based perception models.
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"

#include "SepDynDiskProj.h"
#include "BoxInclusionClassifier.h"
#include "IntervalMonteCarlo.h"

#include <vector>
#include <iostream>



/*==================================================================================
 * TRAJECTORY SETUP
 *==================================================================================*/

// Initial and final time of the trajectory
const double t0 = 0.55;
const double tf = 12;

// Time step between trajectory points
const double dt = 0.01;

// Time domain for the trajectory
const Interval tdomain(t0, tf);

// Temporal definition of the trajectory using TFunction
// Format: (x(t); y(t); heading(t))
TFunction path("( 5*cos(0.5*t)*sin(t) ; 5*cos(0.5*t)*cos(t) ; atan2(-2.5*sin(0.5*t)*cos(t) - 5*cos(0.5*t)*sin(t) , -2.5*sin(0.5*t)*sin(t) + 5*cos(0.5*t)*cos(t)))");

// Generate TrajectoryVector from the path
TrajectoryVector generate_trajectory()
{
    TrajectoryVector traj(tdomain, path, dt);
    traj[2] = traj[2].make_continuous(); // Make heading continuous to handle wrap-around at 0/2pi
    return traj;
}

// Convert TrajectoryVector to TubeVector for contraction
TubeVector generate_tube(TrajectoryVector &traj)
{
    TubeVector tube(traj, dt);
    return tube;
}



/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

// Time period over which to project the sensor coverage
// Here we consider the whole trajectory
Interval t_proj(t0, tf);

// Temporal resolution of the projection (smaller = finer resolution)
double eps_proj = 0.01;



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

// Resolution of the paving (smaller = more refined discretization)
double paving_resolution = 0.2;

// 2D area to be paved (x, y)
IntervalVector paving_area = {{-7, 7}, {-8, 8}};



/*==================================================================================
 * CUSTOM INTERVAL MONTE CARLO
 *==================================================================================*/

// AUV mission sample definition.
//
// Each sample represents a single realization of an AUV mission and includes:
//   - a robot trajectory (obtained via simulation),
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
    // MONTE CARLO SAMPLE GENERATION
    // ----------------------------------------------------

    // Vector containing all Monte Carlo samples.
    // Each sample represents one possible realization of an AUV mission.
    std::vector<AUV_Mission_Sample> samples = {};


    // ############# Sample 1 #############

    // Generate the trajectory and the tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);
    
    // Box containing the object to detect
    IntervalVector box_to_cover = {{2., 3.},{-2.5,-1.5}};
    
    // Detection range of the robot
    double detection_range = 1.5;
    
    // Create the sample
    AUV_Mission_Sample sample = {my_tube, box_to_cover, detection_range};
    
    // Add the sample to the vector
    samples.push_back(sample);

    // ############# Sample 2 #############
    // AUV_Mission_Sample sample2 = {my_tube2, box_to_cover2, detection_range2};
    // samples.push_back(sample2);

    // ...
    // ...
    // ...

    // ############# Sample N #############
    // AUV_Mission_Sample sampleN = {my_tubeN, box_to_coverN, detection_rangeN};
    // samples.push_back(sampleN);

    // Additional samples can be added here to represent other mission
    // realizations (e.g., different trajectories, object positions, or sensors)

    
    // ----------------------------------------------------
    // INTERVAL MONTE CARLO ESTIMATION
    // ----------------------------------------------------

    // Create the custom three-valued logic Monte Carlo estimator
    MyCustomIntervalMonteCarlo my_estimator;

    // Process all samples and update the empirical probability bound
    my_estimator.process_samples(samples);

    // Retrieve the estimated interval bounding the success probability
    Interval estimated_probability = my_estimator.get_probability_bound();

    // For this specific configuration, the object is fully covered,
    // so the expected interval probability is [1,1].
    std::cout<< "\nExpected interval probability: <1, 1> (i.e., Interval(1))"<< std::endl;
    
    std::cout<< "The estimated interval probability is: "<< estimated_probability << std::endl<< std::endl;


    // ----------------------------------------------------
    // VISUALIZATION
    // ----------------------------------------------------

    // Initialize the graphical display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);

    // Define display limits
    fig.axis_limits(-9,9,-9,9);
    fig.draw_box(paving_area, "black");

    // Display the area covered by the sensor along the trajectory
    SepDynDiskProj my_sep(my_tube, detection_range, t_proj, eps_proj, true);
    SIVIA(paving_area, my_sep, paving_resolution);

    // Display the AUV trajectory and its uncertainty tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display the uncertain object position
    fig.draw_box(box_to_cover, "black[brown]");

    // Display the sensor footprint and vehicle pose at selected time instants
    std::vector<double> detection_time = {1.2, 6.5, 11.5};
    for (int i = 0; i < detection_time.size(); i++)
    {
        // Sensor detection disk at the given time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        fig.draw_circle(px, py, detection_range, "black");

        // AUV pose at the given time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.8);
    }
    
    // Render the figure
    fig.show(0);
    vibes::endDrawing();

    return EXIT_SUCCESS;
}