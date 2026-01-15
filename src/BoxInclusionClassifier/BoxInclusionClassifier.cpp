/*
 * @file BoxInclusionClassifier.cpp
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief Implementation of a three-valued logic box inclusion classifier.
 *
 * @details
 * This file implements the methods of the BoxInclusionClassifier class, which evaluates
 * the inclusion of a 2D interval box with respect to a 2D set represented by a CODAC
 * separator.
 *
 * Two classification strategies are provided:
 *   - A fast single-call method for minimal separators.
 *   - A paving-based method adapted from the SIVIA algorithm in CODAC, suitable for
 *     both minimal and non-minimal separators.
 *
 * The classifier returns three-valued logic results encoded as intervals:
 *   [1]   => certainly inside,
 *   [0]   => certainly outside,
 *   [0,1] => uncertain (boundary intersection or unresolved classification).
 *
 * For a full description of the class interface and usage context, see
 * BoxInclusionClassifier.h.
*/

#include "BoxInclusionClassifier.h"
#include <iostream>

namespace
{
    /*
     * @brief Compute the complementary boxes of x inside x0.
     *
     * @details
     * This function computes a list of boxes whose union corresponds to the set
     * difference:
     *
     *     x0 \ x
     *
     * such that:
     *   - the union of the returned boxes and x reconstructs x0,
     *   - the returned boxes do not intersect x.
     *
     * This utility function is directly adapted from the SIVIA paving algorithm
     * implementation in CODAC and is used internally during the paving process.
     *
     * For details on the algorithmic principles, refer to the original CODAC SIVIA
     * implementation.
     *
     * @param x0 Reference box.
     * @param x  Sub-box to remove from x0.
     * @return List of boxes forming the set difference x0 \ x.
     */
    vector<IntervalVector> box_diff(const IntervalVector& x0, const IntervalVector& x)
    {
        vector<IntervalVector> v;
        IntervalVector* boxes;
        int n = x0.diff(x, boxes);
        v.assign(boxes, boxes + n);
        return v;
    }
}

// ============================================================================
// Constructor
// ============================================================================

BoxInclusionClassifier::BoxInclusionClassifier(ibex::Sep& separator): 
_sep(separator) 
{}

// ============================================================================
// Fast inclusion evaluation (minimal separators only)
// ============================================================================

Interval BoxInclusionClassifier::fast_classify(const IntervalVector& box)
{   
    /*
     * This method performs a single separator call to classify the box.
     * It is sound only if the separator is minimal.
     */

    // Enforce 2D assumption (custom separators used in this application are 2D)
    if (box.size()!=2)
    {
        throw std::range_error("[ERROR] BoxInclusionClassifier::fast_classify: The input IntervalVector box must be two-dimensional");
    }

    // Copies of the input box used for inside/outside contractions
    IntervalVector box_in(box);
    IntervalVector box_out(box);

    // Apply separator contraction
    _sep.separate(box_in, box_out);

    /*
     * CODAC naming convention:
     *   - box_in  : points proven to be outside the set
     *   - box_out : points proven to be inside the set
     */

    // If box_in is empty, no point is proven outside => box is fully inside
    if (box_in.is_empty())
    {
        return Interval(1); // certainly true
    }

    // If box_out is empty, no point is proven inside => box is fully outside
    if (box_out.is_empty())
    {
        return Interval(0); // certainly false
    }

    // Otherwise, both inside and outside points may exist
    return Interval(0,1); // uncertain
}

// ============================================================================
// Paving-based inclusion evaluation (minimal and non-minimal separators)
// ============================================================================

Interval BoxInclusionClassifier::classify(const IntervalVector& box, const double paving_resolution, bool overpass_warning)
{
    /*
     * This method implements a paving-based classification adapted from the
     * SIVIA algorithm in CODAC. Only the logic specific to three-valued inclusion
     * classification is commented in detail here.
     */

    // Enforce 2D assumption
    if (box.size()!=2)
    {
        throw std::range_error("[ERROR] BoxInclusionClassifier::classify: The input IntervalVector box must be two-dimensional");
    }

    // Flags used to track detected inclusion states during paving
    bool IN_detected = false; // at least one subdivision fully inside
    bool OUT_detected = false; // at least one subdivision fully outside
    bool UNKNOWN_detected = false; // unresolved subdivision at resolution limit

    // ------------------------------------------------------------------------
    // Paving algorithm (adapted from CODAC SIVIA)
    // ------------------------------------------------------------------------

    ibex::LargestFirst bisector(0.);
    deque<IntervalVector> stack = {box};

    while(!stack.empty())
    {   
        // ------------------------------------------------------------------------
        // Modification from original algorithm:
        // If both inside and outside points are detected, the box intersects the boundary.
        // Early termination: return [0,1] without further paving.
        // This is a logical implication AND an algorithmic optimization.
        // ------------------------------------------------------------------------
        if (IN_detected && OUT_detected)
        {
            return Interval(0,1);
        }

        IntervalVector x_before_ctc = stack.front();
        stack.pop_front();
        
        IntervalVector x_in(x_before_ctc), x_out(x_before_ctc);
        _sep.separate(x_in, x_out);

        // Intersection corresponds to undecided region (original SIVIA logic)
        IntervalVector x = x_in & x_out;

        // Compute fully inside and fully outside regions (original SIVIA logic)
        vector<IntervalVector> x_in_l, x_out_l;
        x_in_l = box_diff(x_before_ctc, x_in);
        x_out_l = box_diff(x_before_ctc, x_out);

        // ------------------------------------------------------------------------
        // Modification from original algorithm:
        // Set boolean variables if a subdivision is detected fully inside/outside
        // ------------------------------------------------------------------------
        if (x_in_l.size()>0)
        {
            IN_detected = true;
        }

        if (x_out_l.size()>0)
        {
            OUT_detected = true;
        }

        // Remaining undecided region
        if(!x.is_empty())
        {
            IntervalVector& x_remaining = x;
            if(x_remaining.max_diam() >= paving_resolution)
            {
                // Subdivide undecided region (original SIVIA logic)
                pair<IntervalVector,IntervalVector> p = bisector.bisect(x_remaining);
                stack.push_back(p.first);
                stack.push_back(p.second);
            }
            // --------------------------------------------------------------------
            // Modification from original algorithm:
            // Reached paving resolution limit without a definitive classification.
            // Mark as UNKNOWN_detected to return [0,1] later.
            // --------------------------------------------------------------------
            else
            {
                UNKNOWN_detected = true;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Final evaluation
    // ------------------------------------------------------------------------

    /*
     * If unresolved regions remain after full paving, the inclusion cannot
     * be decided with certainty.
     */

    if (UNKNOWN_detected)
    {
        if (!overpass_warning)
        {
            std::cout<<"[WARNING] BoxInclusionClassifier::classify(const IntervalVector& box, const double maximal_resolution, bool overpass_warning):f The classifier was unable to remove uncertainty"<<std::endl;
        }
        return Interval(0,1);
    }

    // Only inside points detected
    if (IN_detected)
    {
        return Interval(1);
    }

    // Only outside points detected
    if (OUT_detected)
    {
        return Interval(0);
    }

    // Fallback (should not be reached)
    return Interval(0,1);
}