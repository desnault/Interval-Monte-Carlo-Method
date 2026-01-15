/*
 * @file BoxInclusionClassifier.h
 * @author Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date 2025
 *
 * @brief Three-valued logic inclusion classifier for 2D boxes using CODAC separators.
 *
 * @details
 * This file defines the BoxInclusionClassifier class, which evaluates the inclusion of a
 * two-dimensional interval box with respect to a two-dimensional set represented by a
 * CODAC separator.
 *
 * The classifier follows Kleene’s three-valued logic and returns an Interval encoding
 * the truth value of the proposition:
 *   "the box is covered by the set".
 *
 * The returned values are:
 *   - [1]   : the box is certainly inside the set (fully covered),
 *   - [0]   : the box is certainly outside the set (not covered),
 *   - [0,1] : the inclusion is uncertain, either because the box intersects the boundary
 *             of the set or because the separator and/or numerical resolution cannot
 *             resolve the inclusion unambiguously.
 *
 * This three-valued logic formulation is particularly suited for Monte Carlo methods
 * involving epistemic uncertainty, such as the estimation of mission success probability
 * for robots with stochastic motion and set-based perception models.
 *
 * In the intended application, the separator represents the 2D covered area of a robot
 * along a known trajectory, and the boxes represent uncertain spatial regions (e.g.,
 * object location uncertainty modeled as interval boxes).
 */

#ifndef __BOX_INCLUSION_CLASSIFIER_H___
#define __BOX_INCLUSION_CLASSIFIER_H___

#include "codac.h"

using namespace codac;

/*
 * @class BoxInclusionClassifier
 * @brief Classifier for evaluating inclusion of 2D boxes in a 2D set using CODAC separators.
 *
 * @details
 * The BoxInclusionClassifier provides two methods (`fast_classify` and `classify`) to
 * evaluate whether a given 2D interval box is fully inside, fully outside, or partially
 * inside a set represented by a CODAC separator.
 *
 * Key concepts:
 *   - Three-valued logic (Kleene): Inclusion is encoded as an Interval:
 *       [1]   => box is fully inside (certainly covered),
 *       [0]   => box is fully outside (certainly not covered),
 *       [0,1] => box partially inside or unresolved due to separator/resolution limitations.
 *
 *   - Minimal separator: A separator that, in a single call, can determine the smallest
 *     boxes that contain points inside and outside the set. `fast_classify` is sound
 *     only for minimal separators.
 *
 *   - Non-minimal separator: May require multiple calls or a paving approach (`classify`)
 *     to resolve box inclusion reliably.
 *
 *   - 2D limitation: This class currently supports only 2D boxes and 2D separators, 
 *     as required for the application (e.g., robot coverage along a planar trajectory).
 *     Adapting to N dimensions is straightforward: remove the 2D assertion in
 *     `fast_classify` and `classify`.
 *
 *   - Separator ownership: The classifier does not own the separator; the caller must
 *     ensure the separator remains valid for the lifetime of this class instance.
 *
 * Usage context:
 *   - Designed for integration with Monte Carlo simulations that propagate epistemic
 *     uncertainty using three-valued logic.
 *   - Can also be used in other applications requiring robust box-inclusion evaluation
 *     over 2D sets.
 */

class BoxInclusionClassifier
{
    public:

        /*
         * @brief Constructor
         *
         * @details
         * Initializes a BoxInclusionClassifier with a reference to a CODAC separator that
         * defines the 2D set for box inclusion evaluation.
         *
         * @param separator Reference to the CODAC separator representing the set to classify.
         *
         * @note The classifier does not take ownership of the separator. The caller must ensure
         *       that the separator remains valid for the lifetime of this BoxInclusionClassifier
         *       instance.
         */
        BoxInclusionClassifier(ibex::Sep& separator);

        /*
         * @brief Destructor
         *
         * @details
         * The destructor is defaulted. No dynamic memory is allocated within this class, 
         * and the separator is not owned by the class. Cleaning up the separator, if needed, 
         * is the responsibility of the caller.
         */
        ~BoxInclusionClassifier() = default;

        /*
         * @brief Fast inclusion evaluation for minimal separators.
         *
         * @details
         * Evaluates whether a given 2D box is fully inside, fully outside, or partially inside
         * the 2D set represented by the CODAC separator.
         *
         * This method performs a **single call** to the separator, so it is **sound only for
         * minimal separators**. Minimal separators return the smallest boxes containing points
         * inside and outside the set in one separation step. For non-minimal separators, the
         * result may be overly uncertain, and the `classify` method should be used instead.
         *
         * CODAC naming convention note:
         *   - `box_in` corresponds to points proven **outside** the set,
         *   - `box_out` corresponds to points proven **inside** the set.
         *
         * Three-valued logic output (Kleene):
         *   - [1]   => box is fully inside the set (certainly covered),
         *   - [0]   => box is fully outside the set (certainly not covered),
         *   - [0,1] => box partially inside or unresolved due to separator limitations.
         *
         * @param box 2D interval box to classify.
         *
         * @return Interval encoding the three-valued inclusion logic as described above.
         *
         * @throws std::range_error if the input box is not two-dimensional.
         *
         * @note This method is particularly suited for Monte Carlo simulations where the
         *       three-valued logic output will be used to propagate epistemic uncertainty.
         */
        Interval fast_classify(const IntervalVector& box);

        /*
         * @brief Paving-based inclusion evaluation for minimal and non-minimal separators.
         *
         * @details
         * Evaluates whether a given 2D box is fully inside, fully outside, or partially inside
         * the 2D set represented by the CODAC separator, using a SIVIA-like paving algorithm.
         *
         * Unlike `fast_classify`, which performs a single separator call and is only sound
         * for minimal separators, this method subdivides the box until the **paving resolution**
         * is reached, allowing reliable classification for both minimal and non-minimal separators.
         *
         * The paving algorithm:
         *   - Starts with the full box on a stack.
         *   - Iteratively pops a box, applies the separator, and splits it if necessary.
         *   - Detects subdivisions that are fully inside, fully outside, or partially inside/outside.
         *   - Uses early termination: if both inside and outside points are detected, the box is
         *     immediately classified as partially inside/outside ([0,1]), which is both a
         *     **logical implication** and an **algorithmic optimization**.
         *
         * Boolean flags used during paving:
         *   - IN_detected      : true if a subdivision is fully inside the set.
         *   - OUT_detected     : true if a subdivision is fully outside the set.
         *   - UNKNOWN_detected : true if a subdivision is partially inside/outside or resolution
         *                       limit is reached without definitive classification.
         *
         * Three-valued logic output (Kleene):
         *   - [1]   => box is fully inside (certainly covered),
         *   - [0]   => box is fully outside (certainly not covered),
         *   - [0,1] => box partially inside or unresolved due to separator limitations or
         *             insufficient paving resolution.
         *
         * CODAC naming convention note:
         *   - `box_in` corresponds to points proven **outside** the set,
         *   - `box_out` corresponds to points proven **inside** the set.
         *
         * @param box               2D interval box to classify.
         * @param paving_resolution  Maximum allowed diameter of box subdivisions.
         *                          Smaller values yield finer classification but increase computation.
         * @param overpass_warning   If false (default), prints a warning when the box inclusion
         *                          cannot be resolved completely. If true, suppresses messages.
         *
         * @return Interval encoding three-valued inclusion logic as described above.
         *
         * @throws std::range_error if the input box is not two-dimensional.
         *
         * @note This method is particularly suited for Monte Carlo simulations that propagate
         *       epistemic uncertainty using three-valued logic. For unexperienced users, a
         *       recommended paving resolution is proportional to box size, e.g.:
         *           paving_resolution = box.max_diam() / 10
         *       This ensures a good trade-off between accuracy and computation time.
         */
        Interval classify(const IntervalVector& box, const double paving_resolution, bool overpass_warning = false);
    
    private:

        /*
         * @brief Reference to the CODAC separator defining the 2D set.
         *
         * @details
         * This separator is used to evaluate box inclusion. It represents the 2D covered
         * area of a robot or any other 2D set. 
         *
         * Note:
         *   - The classifier does **not** own the separator. The caller must ensure that
         *     the separator remains valid for the lifetime of this BoxInclusionClassifier instance.
         *   - Separator methods are used internally to classify boxes as inside, outside,
         *     or partially inside the set.
         */
        ibex::Sep& _sep;
};

# endif