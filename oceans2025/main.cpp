/*
 * @file main.cpp
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief OCEANS 2025 example: interval Monte Carlo evaluation of a multi-object
 * inspection mission conducted by an autonomous underwater vehicle.
 *
 * @details
 * This script illustrates the use of the IntervalMonteCarlo and
 * BoxInclusionClassifier classes on a realistic mission scenario.
 *
 * In this example, an inspection AUV follows a planned trajectory in order to
 * observe three objects lying on the seabed. The true vehicle behavior is
 * stochastic, and a set of simulated trajectories has been generated beforehand
 * and exported in the `data/oceans2025/Noisy` folder.
 *
 * Each object is represented by an uncertain 2D bounding box on the seabed.
 * For a given simulated trajectory, the mission outcome is evaluated using
 * three-valued logic:
 *
 *   - TRUE    : the object box is guaranteed to be fully covered by the sensor,
 *   - FALSE   : the object box is guaranteed not to be covered,
 *   - UNKNOWN : the object coverage is uncertain.
 *
 * Four empirical interval probability bounds are estimated:
 *
 *   - probability to see box 1,
 *   - probability to see box 2,
 *   - probability to see box 3,
 *   - probability to see all three boxes in the same mission.
 *
 * The script:
 *
 *   1. loads all noisy simulated trajectories exported as TubeVector objects,
 *   2. evaluates each trajectory with interval-based classification,
 *   3. computes the evolution of the four probability bounds,
 *   4. exports these results for later visualization in `result.py`,
 *   5. exports the global three-valued result associated with each trajectory.
 *
 * Notes:
 * - This script is intended as a publication/example scenario rather than as a
 *   generic reusable tool.
 * - The input trajectories are expected to have been generated beforehand by the
 *   Python script `data/main.py`.
 */


/*==============================================================================
 * LIBRARY IMPORT
 *============================================================================*/

#include "codac.h"

#include "SepDynDiskProj.h"
#include "BoxInclusionClassifier.h"
#include "IntervalMonteCarlo.h"

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <string>
#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <iomanip>



/*==============================================================================
 * SCENARIO DEFINITIONS
 *============================================================================*/

/**
 * @brief Sample type used by the IntervalMonteCarlo framework.
 *
 * Each sample corresponds to one simulated AUV mission and contains:
 *   - the vehicle trajectory as a codac::TubeVector,
 *   - the filename of the trajectory (used for traceability and export).
 */
struct Oceans2025Sample
{
	codac::TubeVector trajectory;   ///< Simulated vehicle trajectory
	std::string filename;           ///< Associated file name (for identification)
};

/**
 * @brief Estimation modes for the Monte Carlo estimator.
 *
 * The same estimator class is reused with different modes in order to compute:
 *   - the probability to detect each individual object,
 *   - the probability to detect all objects simultaneously.
 */
enum class EstimationMode
{
	BOX1,       ///< Detection of object 1 only
	BOX2,       ///< Detection of object 2 only
	BOX3,       ///< Detection of object 3 only
	ALL_BOXES   ///< Joint detection of the three objects
};

/**
 * @brief Sensor and projection parameters.
 *
 * - detection_range: sensing radius of the AUV (in meters).
 * - eps_proj: temporal resolution used in the dynamic projection of the sensor
 *   footprint along the trajectory.
 */
const double detection_range = 2.0;
const double eps_proj = 0.01;

/**
 * @brief Uncertain object positions (2D interval boxes).
 *
 * Each object is modeled as a box on the seabed. The true position of the object
 * is unknown but guaranteed to lie within its associated box.
 */
const codac::IntervalVector box1{{9.5, 10.5}, {1.2, 2.2}};
const codac::IntervalVector box2{{23.0, 24.0}, {11.5, 12.5}};
const codac::IntervalVector box3{{43.0, 44.0}, {28.0, 29.0}};


/*==============================================================================
 * INTERVAL MONTE CARLO ESTIMATOR
 *============================================================================*/

/**
 * @brief Interval Monte Carlo estimator specialized for the OCEANS 2025 scenario.
 *
 * This class derives from IntervalMonteCarlo and defines how a single trajectory
 * (sample) is evaluated using three-valued logic.
 *
 * Depending on the selected EstimationMode, the estimator evaluates:
 *   - the detection of a single object (box1, box2, or box3),
 *   - or the joint detection of all three objects.
 *
 * The classification is based on:
 *   - a dynamic separator (SepDynDiskProj) representing the sensor footprint
 *     along the trajectory,
 *   - a BoxInclusionClassifier that evaluates whether each object box is fully
 *     covered, not covered, or partially covered.
 */
class Oceans2025Estimator : public IntervalMonteCarlo<Oceans2025Sample>
{
	public:

		/**
		 * @brief Constructor.
		 *
		 * @param mode Estimation mode (single box or all boxes).
		 */
		Oceans2025Estimator(EstimationMode mode): m_mode(mode){}

		/**
		 * @brief Classify a trajectory using three-valued logic.
		 *
		 * @param sample A simulated AUV mission.
		 * @return THREE_VALUED_LOGIC classification result.
		 *
		 * Logic:
		 * - For single-box modes: directly classify the corresponding box.
		 * - For ALL_BOXES:
		 *     FALSE   if at least one box is FALSE,
		 *     UNKNOWN if no FALSE but at least one UNKNOWN,
		 *     TRUE    if all boxes are TRUE.
		 */
		THREE_VALUED_LOGIC classify(const Oceans2025Sample& sample) const override
		{
			// Extract trajectory time domain
			codac::Interval trajectory_time_domain = sample.trajectory.tdomain();

			// Extract position components (x, y)
			codac::TubeVector position_trajectory = sample.trajectory.subvector(0,1);

			// Build dynamic separator representing sensor coverage
			SepDynDiskProj separator(position_trajectory, detection_range, trajectory_time_domain, eps_proj, true);

			// Build classifier associated with this separator
			BoxInclusionClassifier classifier(separator);

			// --- Single box evaluation ---
			if (m_mode == EstimationMode::BOX1)
			{
				return classify_box(classifier, box1);
			}
			else if (m_mode == EstimationMode::BOX2)
			{
				return classify_box(classifier, box2);
			}
			else if (m_mode == EstimationMode::BOX3)
			{
				return classify_box(classifier, box3);
			}

			// --- Joint evaluation (ALL_BOXES) ---
			else
			{
				THREE_VALUED_LOGIC result_box1 = classify_box(classifier, box1);
				THREE_VALUED_LOGIC result_box2 = classify_box(classifier, box2);
				THREE_VALUED_LOGIC result_box3 = classify_box(classifier, box3);

				// Kleene AND logic
				if ((result_box1 == THREE_VALUED_LOGIC::FALSE) ||
					(result_box2 == THREE_VALUED_LOGIC::FALSE) ||
					(result_box3 == THREE_VALUED_LOGIC::FALSE))
				{
					return THREE_VALUED_LOGIC::FALSE;
				}

				if ((result_box1 == THREE_VALUED_LOGIC::UNKNOWN) ||
					(result_box2 == THREE_VALUED_LOGIC::UNKNOWN) ||
					(result_box3 == THREE_VALUED_LOGIC::UNKNOWN))
				{
					return THREE_VALUED_LOGIC::UNKNOWN;
				}

				return THREE_VALUED_LOGIC::TRUE;
			}
		}

	private:

		EstimationMode m_mode; ///< Selected estimation mode

		/**
		 * @brief Classify a single object box using interval inclusion.
		 *
		 * @param classifier BoxInclusionClassifier instance.
		 * @param box Object bounding box.
		 * @return THREE_VALUED_LOGIC classification result.
		 *
		 * The classifier returns an interval:
		 *   [1,1] -> TRUE
		 *   [0,0] -> FALSE
		 *   otherwise -> UNKNOWN
		 */
		THREE_VALUED_LOGIC classify_box(BoxInclusionClassifier& classifier, const codac::IntervalVector& box) const
		{
			codac::Interval result = classifier.classify(box, box.max_diam() / 10.0, true);

			if (result == codac::Interval(1.0))
			{
				return THREE_VALUED_LOGIC::TRUE;
			}
			else if (result == codac::Interval(0.0))
			{
				return THREE_VALUED_LOGIC::FALSE;
			}
			else
			{
				return THREE_VALUED_LOGIC::UNKNOWN;
			}
		}
};


/*==============================================================================
 * HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Convert a THREE_VALUED_LOGIC value to a string.
 *
 * @param value Three-valued logic result.
 * @return Corresponding string ("TRUE", "FALSE", or "UNKNOWN").
 */
std::string logic_to_string(THREE_VALUED_LOGIC value)
{
	if (value == THREE_VALUED_LOGIC::TRUE)
	{
		return "TRUE";
	}
	else if (value == THREE_VALUED_LOGIC::FALSE)
	{
		return "FALSE";
	}
	else
	{
		return "UNKNOWN";
	}
}

/**
 * @brief Load all TubeVector trajectories from a directory.
 *
 * @param directory_path Path to the directory containing ".tubevector" files.
 * @return Vector of Oceans2025Sample objects.
 *
 * The function:
 *   1. scans the directory for ".tubevector" files,
 *   2. sorts them to ensure deterministic processing order,
 *   3. loads each trajectory and stores it with its filename.
 *
 * Notes:
 * - Only regular files with ".tubevector" extension are considered.
 * - The filename is kept to allow traceability when exporting results.
 */
std::vector<Oceans2025Sample> load_samples_in_directory(const std::string& directory_path)
{
	std::vector<Oceans2025Sample> samples;
	std::vector<std::filesystem::path> tubevector_files;

	// Scan directory for TubeVector files
	for (const auto& entry : std::filesystem::directory_iterator(directory_path))
	{
		if (entry.is_regular_file() && entry.path().extension() == ".tubevector")
		{
			tubevector_files.push_back(entry.path());
		}
	}

	// Sort files to ensure reproducible order
	std::sort(tubevector_files.begin(), tubevector_files.end());

	// Load each trajectory
	for (const auto& file_path : tubevector_files)
	{
		codac::TubeVector trajectory(file_path.string());

		Oceans2025Sample sample = {trajectory, file_path.filename().string()};
		samples.push_back(sample);
	}

	return samples;
}


/*==============================================================================
 * MAIN
 *============================================================================*/

int main()
{
	// -------------------------------------------------------------------------
	// DATA DIRECTORY
	// -------------------------------------------------------------------------

	// Path to the data directory (defined via CMake)
	std::string data_directory_path = INTERVAL_MC_DATA_DIR;

	// Path to noisy trajectories
	std::string noisy_directory_path = data_directory_path + "/oceans2025/Noisy";


	// -------------------------------------------------------------------------
	// LOAD MONTE CARLO SAMPLES
	// -------------------------------------------------------------------------

	std::vector<Oceans2025Sample> samples = load_samples_in_directory(noisy_directory_path);

	if (samples.empty())
	{
		throw std::runtime_error("No '.tubevector' file was found in data/oceans2025/Noisy.");
	}

	std::cout << "[INFO] Number of noisy trajectories loaded: " << samples.size() << std::endl;


	// -------------------------------------------------------------------------
	// ESTIMATOR INITIALIZATION
	// -------------------------------------------------------------------------

	// Four estimators corresponding to the four probability bounds
	Oceans2025Estimator estimator_box1(EstimationMode::BOX1);
	Oceans2025Estimator estimator_box2(EstimationMode::BOX2);
	Oceans2025Estimator estimator_box3(EstimationMode::BOX3);
	Oceans2025Estimator estimator_all(EstimationMode::ALL_BOXES);

	// -------------------------------------------------------------------------
	// MONTE CARLO PROCESSING
	// -------------------------------------------------------------------------

	// Store evolution of probability bounds
	std::vector<codac::Interval> probability_box1_evolution;
	std::vector<codac::Interval> probability_box2_evolution;
	std::vector<codac::Interval> probability_box3_evolution;
	std::vector<codac::Interval> probability_all_evolution;

	// Store global classification result for each trajectory
	std::vector<std::pair<std::string, THREE_VALUED_LOGIC>> global_results;

	std::cout << "[INFO] Processing simulation(s) (This might take some time...):" << std::endl;

	size_t i = 0;

	// Process each sample sequentially
	for (const auto& sample : samples)
	{
		// Update all estimators with the current sample
		estimator_box1.process_sample(sample);
		estimator_box2.process_sample(sample);
		estimator_box3.process_sample(sample);
		estimator_all.process_sample(sample);

		// Store current probability bounds
		probability_box1_evolution.push_back(estimator_box1.get_probability_bound());
		probability_box2_evolution.push_back(estimator_box2.get_probability_bound());
		probability_box3_evolution.push_back(estimator_box3.get_probability_bound());
		probability_all_evolution.push_back(estimator_all.get_probability_bound());

		// Compute and store global classification result
		THREE_VALUED_LOGIC global_result = estimator_all.classify(sample);
		global_results.push_back({sample.filename, global_result});

		// Progress display
		i++;
		std::cout << "\r\t" << i << "/" << samples.size() << "      ";
	}

	std::cout << std::endl;

	// -------------------------------------------------------------------------
	// FINAL RESULTS DISPLAY
	// -------------------------------------------------------------------------

	// Retrieve final probability bounds after all samples have been processed
	codac::Interval probability_box1 = estimator_box1.get_probability_bound();
	codac::Interval probability_box2 = estimator_box2.get_probability_bound();
	codac::Interval probability_box3 = estimator_box3.get_probability_bound();
	codac::Interval probability_all  = estimator_all.get_probability_bound();

	// Display results in terminal
	std::cout << std::endl;
	std::cout << "[RESULT] Probability to see box 1: " << probability_box1 << std::endl;
	std::cout << "[RESULT] Probability to see box 2: " << probability_box2 << std::endl;
	std::cout << "[RESULT] Probability to see box 3: " << probability_box3 << std::endl;
	std::cout << "[RESULT] Probability to see all boxes: " << probability_all << std::endl;
	std::cout << std::endl;
	
	// -------------------------------------------------------------------------
	// EXPORT RESULTS
	// -------------------------------------------------------------------------

	// Output directory (same scenario folder)
	std::string output_directory_path = data_directory_path + "/oceans2025";

	// Define output file paths
	std::string probability_box1_filename = output_directory_path + "/prob_box1.txt";
	std::string probability_box2_filename = output_directory_path + "/prob_box2.txt";
	std::string probability_box3_filename = output_directory_path + "/prob_box3.txt";
	std::string probability_all_filename  = output_directory_path + "/prob_all.txt";
	std::string classification_filename   = output_directory_path + "/trajectory_results.txt";

	// Open output files
	std::ofstream probability_box1_file(probability_box1_filename);
	std::ofstream probability_box2_file(probability_box2_filename);
	std::ofstream probability_box3_file(probability_box3_filename);
	std::ofstream probability_all_file(probability_all_filename);
	std::ofstream classification_file(classification_filename);

	// Check file opening
	if (!probability_box1_file.is_open())
	{
		throw std::runtime_error("Unable to create probability file for box 1.");
	}
	if (!probability_box2_file.is_open())
	{
		throw std::runtime_error("Unable to create probability file for box 2.");
	}
	if (!probability_box3_file.is_open())
	{
		throw std::runtime_error("Unable to create probability file for box 3.");
	}
	if (!probability_all_file.is_open())
	{
		throw std::runtime_error("Unable to create probability file for all boxes.");
	}
	if (!classification_file.is_open())
	{
		throw std::runtime_error("Unable to create trajectory classification file.");
	}

	// Set numerical precision for probability exports
	const int precision = 6;

	probability_box1_file << std::fixed << std::setprecision(precision);
	probability_box2_file << std::fixed << std::setprecision(precision);
	probability_box3_file << std::fixed << std::setprecision(precision);
	probability_all_file  << std::fixed << std::setprecision(precision);

	// -------------------------------------------------------------------------
	// WRITE PROBABILITY EVOLUTION
	// -------------------------------------------------------------------------

	// Each line format:
	// N [lower_bound, upper_bound]
	for (size_t i = 0; i < probability_box1_evolution.size(); i++)
	{
		probability_box1_file << (i + 1) << " [" << probability_box1_evolution[i].lb() << "," << probability_box1_evolution[i].ub() << "]" << std::endl;
		probability_box2_file << (i + 1) << " [" << probability_box2_evolution[i].lb() << "," << probability_box2_evolution[i].ub() << "]" << std::endl;
		probability_box3_file << (i + 1) << " [" << probability_box3_evolution[i].lb() << "," << probability_box3_evolution[i].ub() << "]" << std::endl;
		probability_all_file  << (i + 1) << " [" << probability_all_evolution[i].lb()  << "," << probability_all_evolution[i].ub()  << "]" << std::endl;
	}

	// -------------------------------------------------------------------------
	// WRITE TRAJECTORY CLASSIFICATION RESULTS
	// -------------------------------------------------------------------------

	// Each line format:
	// filename THREE_VALUED_LOGIC
	for (const auto& result : global_results)
	{
		classification_file << result.first << " " << logic_to_string(result.second) << std::endl;
	}

	// -------------------------------------------------------------------------
	// FINALIZE EXPORT
	// -------------------------------------------------------------------------

	probability_box1_file.close();
	probability_box2_file.close();
	probability_box3_file.close();
	probability_all_file.close();
	classification_file.close();

	std::cout << "[EXPORT] Probability evolution files exported successfully." << std::endl;
	std::cout << "[EXPORT] Trajectory classification file exported successfully." << std::endl;

	return EXIT_SUCCESS;
}