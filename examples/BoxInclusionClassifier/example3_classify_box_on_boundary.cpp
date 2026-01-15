/********************************************************************************************
 * @file    example3_classify_box_on_boundary.cpp
 * @author  Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date    2025
 * @brief   Example illustrating the BoxInclusionClassifier for a box partially inside the
 *          robot-covered area.
 *
 * @details
 * This example demonstrates the use of BoxInclusionClassifier with a 2D separator
 * representing the coverage of a robot along a known trajectory. The goal is to show
 * that the classifier correctly identifies a box that is partially inside the covered area.
 *
 * Both `fast_classify()` and `classify()` methods are evaluated, and the results are
 * displayed using VIBes. The figure highlights:
 *   - The robot trajectory and tube
 *   - The covered area
 *   - The box being classified
 *   - Perception circles at selected times
 *
 * Expected output:
 *   - The inclusion classifier should return [0, 1] for this box (partially inside / uncertain).
 ********************************************************************************************/

#include "codac.h"
#include "vibes.h"
#include "SepDynDiskProj.h"
#include "BoxInclusionClassifier.h"

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
 * DETECTION PARAMETERS
 *==================================================================================*/

// Binary sensor detection range (disk radius)
double detection_range = 1.5;



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



int main()
{
    // Generate the trajectory and the associated tube
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Create CODAC separator
    SepDynDiskProj my_sep(my_tube, detection_range, t_proj, eps_proj, true);

    // Create BoxInclusionClassifier instance
    BoxInclusionClassifier my_classifier(my_sep);

    // Define a box to classify
	IntervalVector box_to_classify= {{-1.7, -0.7},{-6.8, -5.8}};
	std::cout<<"In this example, the box (represented in brown on the figure) is partially inside the covered area (colored in green on the figure).\nExpected output of the inclusion classifier: [0, 1] (Uncertain)\n"<<std::endl;

    // Evaluate inclusion using fast_classify() method
    Interval evaluation_result = my_classifier.fast_classify(box_to_classify);
    std::cout<<"Result returned by the fast_classify() method: "<<evaluation_result<<std::endl;

    // Evaluate inclusion using classify() method
    evaluation_result = my_classifier.classify(box_to_classify, box_to_classify.max_diam()/10.);
    std::cout<<"Result returned by the classify() method: "<<evaluation_result<<std::endl<<std::endl;

    // Start display
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100,100,800,800);
    fig.axis_limits(-9,9,-9,9);
    fig.draw_box(paving_area, "black");
    
    // Display covered area
    SIVIA(paving_area, my_sep, paving_resolution);

    // Display robot trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display box to cover (assume to contain an object)
    fig.draw_box(box_to_classify, "black[brown]");

    // Display AUV and associated perception at three different time
    std::vector<double> detection_time = {1., 6.5, 11.5};
    for (int i=0; i<detection_time.size(); i++)
    {
        // Display the detection range at the considered time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        fig.draw_circle(px, py, detection_range, "black");

        // Display AUV position at the considered time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.8);
    }
    
    // Do not display the robot at the end of the trajectory
    fig.show(0);
    
    // End display
    vibes::endDrawing();

    return EXIT_SUCCESS;
}