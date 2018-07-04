#ifndef slic3r_ClipperUtils_hpp_
#define slic3r_ClipperUtils_hpp_

#include <libslic3r.h>
#include "clipper.hpp"
#include "ExPolygon.hpp"
#include "Polygon.hpp"
#include "Surface.hpp"

// import these wherever we're included
using ClipperLib::jtMiter;
using ClipperLib::jtRound;
using ClipperLib::jtSquare;

namespace Slic3r {

#define CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR (0.005f)
#define CLIPPER_OFFSET_SCALE_ROUNDING_DELTA ((1 << (CLIPPER_OFFSET_POWER_OF_2 - 1)) - 1)

// Factor to convert from coord_t (which is int32) to an int64 type used by the Clipper library.
//FIXME Vojtech: Better to use a power of 2 coefficient and to use bit shifts for scaling.
// How about 2^17=131072?
// By the way, is the scalling needed at all? Cura runs all the computation with a fixed point precision of 1um, while Slic3r scales to 1nm,
// further scaling by 10e5 brings us to 
constexpr auto CLIPPER_OFFSET_POWER_OF_2 = 17;
constexpr float CLIPPER_OFFSET_SCALE = (1 << CLIPPER_OFFSET_POWER_OF_2);
constexpr auto MAX_COORD = ClipperLib::hiRange / CLIPPER_OFFSET_SCALE;

//-----------------------------------------------------------
// legacy code from Clipper documentation
void AddOuterPolyNodeToExPolygons(ClipperLib::PolyNode& polynode, Slic3r::ExPolygons& expolygons);
void PolyTreeToExPolygons(ClipperLib::PolyTree& polytree, Slic3r::ExPolygons& expolygons);
//-----------------------------------------------------------

ClipperLib::Path Slic3rMultiPoint_to_ClipperPath(const Slic3r::MultiPoint &input);
template <class T>
ClipperLib::Paths Slic3rMultiPoints_to_ClipperPaths(const T &input);
template <class T>
T ClipperPath_to_Slic3rMultiPoint(const ClipperLib::Path &input);
template <class T>
T ClipperPaths_to_Slic3rMultiPoints(const ClipperLib::Paths &input);
Slic3r::ExPolygons ClipperPaths_to_Slic3rExPolygons(const ClipperLib::Paths &input);

void scaleClipperPolygons(ClipperLib::Paths &polygons, const double scale);

// offset Polygons
ClipperLib::Paths _offset(const Slic3r::Polygons &polygons, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);
Slic3r::Polygons offset(const Slic3r::Polygons &polygons, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);

// offset Polylines
ClipperLib::Paths _offset(const Slic3r::Polylines &polylines, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtSquare, 
    double miterLimit = 3);
Slic3r::Polygons offset(const Slic3r::Polylines &polylines, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtSquare, 
    double miterLimit = 3);

// TODO @Samir55 out it in .cpp file.
void unscaleClipperPolygons(ClipperLib::Paths &polygons)
{
    for (ClipperLib::Paths::iterator it = polygons.begin(); it != polygons.end(); ++it)
        for (ClipperLib::Path::iterator pit = (*it).begin(); pit != (*it).end(); ++pit) {
            pit->X += CLIPPER_OFFSET_SCALE_ROUNDING_DELTA;
            pit->Y += CLIPPER_OFFSET_SCALE_ROUNDING_DELTA;
            pit->X >>= CLIPPER_OFFSET_POWER_OF_2;
            pit->Y >>= CLIPPER_OFFSET_POWER_OF_2;
        }
}

ClipperLib::Path
Slic3rMultiPoint_to_ClipperPath_reversed(const Slic3r::MultiPoint &input)
{
    ClipperLib::Path output;
    output.reserve(input.points.size());
    for (Slic3r::Points::const_reverse_iterator pit = input.points.rbegin(); pit != input.points.rend(); ++pit)
        output.push_back(ClipperLib::IntPoint( (*pit).x, (*pit).y ));
    return output;
}

void scaleClipperPolygon(ClipperLib::Path &polygon)
{
    for (ClipperLib::Path::iterator pit = polygon.begin(); pit != polygon.end(); ++pit) {
        pit->X <<= CLIPPER_OFFSET_POWER_OF_2;
        pit->Y <<= CLIPPER_OFFSET_POWER_OF_2;
    }
}

void scaleClipperPolygons(ClipperLib::Paths &polygons)
{
    for (ClipperLib::Paths::iterator it = polygons.begin(); it != polygons.end(); ++it)
        for (ClipperLib::Path::iterator pit = (*it).begin(); pit != (*it).end(); ++pit) {
            pit->X <<= CLIPPER_OFFSET_POWER_OF_2;
            pit->Y <<= CLIPPER_OFFSET_POWER_OF_2;
        }
}

ClipperLib::Paths _offset(ClipperLib::Paths &&input, ClipperLib::EndType endType, const float delta, ClipperLib::JoinType joinType, double miterLimit)
{
    // scale input
    scaleClipperPolygons(input);

    // perform offset
    ClipperLib::ClipperOffset co;
    if (joinType == jtRound)
        co.ArcTolerance = miterLimit;
    else
        co.MiterLimit = miterLimit;
    float delta_scaled = delta * float(CLIPPER_OFFSET_SCALE);
    co.shortestEdgeLength = double(std::abs(delta_scaled * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR));
    co.AddPaths(input, joinType, endType);
    ClipperLib::Paths retval;
    co.Execute(retval, delta_scaled);

    // unscale output
    unscaleClipperPolygons(retval);
    return retval;
}

// This is a safe variant of the polygons offset, tailored for multiple ExPolygons.
// It is required, that the input expolygons do not overlap and that the holes of each ExPolygon don't intersect with their respective outer contours.
// Each ExPolygon is offsetted separately, then the offsetted ExPolygons are united.
ClipperLib::Paths _offset(const Slic3r::ExPolygons &expolygons, const float delta,
                          ClipperLib::JoinType joinType, double miterLimit)
{
    const float delta_scaled = delta * float(CLIPPER_OFFSET_SCALE);
    // Offsetted ExPolygons before they are united.
    ClipperLib::Paths contours_cummulative;
    contours_cummulative.reserve(expolygons.size());
    // How many non-empty offsetted expolygons were actually collected into contours_cummulative?
    // If only one, then there is no need to do a final union.
    size_t expolygons_collected = 0;
    for (Slic3r::ExPolygons::const_iterator it_expoly = expolygons.begin(); it_expoly != expolygons.end(); ++ it_expoly) {
        // 1) Offset the outer contour.
        ClipperLib::Paths contours;
        {
            ClipperLib::Path input = Slic3rMultiPoint_to_ClipperPath(it_expoly->contour);
            scaleClipperPolygon(input);
            ClipperLib::ClipperOffset co;
            if (joinType == jtRound)
                co.ArcTolerance = miterLimit * double(CLIPPER_OFFSET_SCALE);
            else
                co.MiterLimit = miterLimit;
            // TODO @Samir55 update Clipper lib.
            co.shortestEdgeLength = double(std::abs(delta_scaled * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR));
            co.AddPath(input, joinType, ClipperLib::etClosedPolygon);
            co.Execute(contours, delta_scaled);
        }
        if (contours.empty())
            // No need to try to offset the holes.
            continue;

        if (it_expoly->holes.empty()) {
            // No need to subtract holes from the offsetted expolygon, we are done.
            contours_cummulative.insert(contours_cummulative.end(), contours.begin(), contours.end());
            ++ expolygons_collected;
        } else {
            // 2) Offset the holes one by one, collect the offsetted holes.
            ClipperLib::Paths holes;
            {
                for (Polygons::const_iterator it_hole = it_expoly->holes.begin(); it_hole != it_expoly->holes.end(); ++ it_hole) {
                    ClipperLib::Path input = Slic3rMultiPoint_to_ClipperPath_reversed(*it_hole);
                    scaleClipperPolygon(input);
                    ClipperLib::ClipperOffset co;
                    if (joinType == jtRound)
                        co.ArcTolerance = miterLimit * double(CLIPPER_OFFSET_SCALE);
                    else
                        co.MiterLimit = miterLimit;
                    co.shortestEdgeLength = double(std::abs(delta_scaled * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR));
                    co.AddPath(input, joinType, ClipperLib::etClosedPolygon);
                    ClipperLib::Paths out;
                    co.Execute(out, - delta_scaled);
                    holes.insert(holes.end(), out.begin(), out.end());
                }
            }

            // 3) Subtract holes from the contours.
            if (holes.empty()) {
                // No hole remaining after an offset. Just copy the outer contour.
                contours_cummulative.insert(contours_cummulative.end(), contours.begin(), contours.end());
                ++ expolygons_collected;
            } else if (delta < 0) {
                // Negative offset. There is a chance, that the offsetted hole intersects the outer contour.
                // Subtract the offsetted holes from the offsetted contours.
                ClipperLib::Clipper clipper;
                clipper.Clear();
                clipper.AddPaths(contours, ClipperLib::ptSubject, true);
                clipper.AddPaths(holes, ClipperLib::ptClip, true);
                ClipperLib::Paths output;
                clipper.Execute(ClipperLib::ctDifference, output, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
                if (! output.empty()) {
                    contours_cummulative.insert(contours_cummulative.end(), output.begin(), output.end());
                    ++ expolygons_collected;
                } else {
                    // The offsetted holes have eaten up the offsetted outer contour.
                }
            } else {
                // Positive offset. As long as the Clipper offset does what one expects it to do, the offsetted hole will have a smaller
                // area than the original hole or even disappear, therefore there will be no new intersections.
                // Just collect the reversed holes.
                contours_cummulative.reserve(contours.size() + holes.size());
                contours_cummulative.insert(contours_cummulative.end(), contours.begin(), contours.end());
                // Reverse the holes in place.
                for (size_t i = 0; i < holes.size(); ++ i)
                    std::reverse(holes[i].begin(), holes[i].end());
                contours_cummulative.insert(contours_cummulative.end(), holes.begin(), holes.end());
                ++ expolygons_collected;
            }
        }
    }

    // 4) Unite the offsetted expolygons.
    ClipperLib::Paths output;
    if (expolygons_collected > 1 && delta > 0) {
        // There is a chance that the outwards offsetted expolygons may intersect. Perform a union.
        ClipperLib::Clipper clipper;
        clipper.Clear();
        clipper.AddPaths(contours_cummulative, ClipperLib::ptSubject, true);
        clipper.Execute(ClipperLib::ctUnion, output, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    } else {
        // Negative offset. The shrunk expolygons shall not mutually intersect. Just copy the output.
        output = std::move(contours_cummulative);
    }

    // 4) Unscale the output.
    unscaleClipperPolygons(output);
    return output;
}

Slic3r::Polygon ClipperPath_to_Slic3rPolygon(const ClipperLib::Path &input)
{
    Polygon retval;
    for (ClipperLib::Path::const_iterator pit = input.begin(); pit != input.end(); ++pit)
        retval.points.push_back(Point( (*pit).X, (*pit).Y ));
    return retval;
}

Slic3r::Polygons ClipperPaths_to_Slic3rPolygons(const ClipperLib::Paths &input)
{
    Slic3r::Polygons retval;
    retval.reserve(input.size());
    for (ClipperLib::Paths::const_iterator it = input.begin(); it != input.end(); ++it)
        retval.push_back(ClipperPath_to_Slic3rPolygon(*it));
    return retval;
}

inline Slic3r::Polygons offset(const Slic3r::ExPolygons &expolygons, const float delta, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miterLimit = 3)
{ return ClipperPaths_to_Slic3rPolygons(_offset(expolygons, delta, joinType, miterLimit)); }

inline Slic3r::Polygons offset(const Slic3r::Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit = 3)
{ return ClipperPaths_to_Slic3rPolygons(_offset(Slic3rMultiPoints_to_ClipperPaths(polygons), ClipperLib::etClosedPolygon, delta, joinType, miterLimit)); }

inline Slic3r::Polygons offset(const Slic3r::Polylines &polylines, const float delta, ClipperLib::JoinType joinType, double miterLimit = 3)
{ return ClipperPaths_to_Slic3rPolygons(_offset(Slic3rMultiPoints_to_ClipperPaths(polylines), ClipperLib::etOpenButt, delta, joinType, miterLimit)); }

Slic3r::Surfaces offset(const Slic3r::Surface &surface, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtSquare, 
    double miterLimit = 3);

Slic3r::ExPolygons offset_ex(const Slic3r::Polygons &polygons, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);
Slic3r::ExPolygons offset_ex(const Slic3r::ExPolygons &expolygons, const float delta,
    double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);

ClipperLib::Paths _offset2(const Slic3r::Polygons &polygons, const float delta1,
    const float delta2, double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);
Slic3r::Polygons offset2(const Slic3r::Polygons &polygons, const float delta1,
    const float delta2, double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);
Slic3r::ExPolygons offset2_ex(const Slic3r::Polygons &polygons, const float delta1,
    const float delta2, double scale = CLIPPER_OFFSET_SCALE, ClipperLib::JoinType joinType = ClipperLib::jtMiter, 
    double miterLimit = 3);

template <class T>
T _clipper_do(ClipperLib::ClipType clipType, const Slic3r::Polygons &subject, 
    const Slic3r::Polygons &clip, const ClipperLib::PolyFillType fillType, bool safety_offset_ = false);

ClipperLib::PolyTree _clipper_do(ClipperLib::ClipType clipType, const Slic3r::Polylines &subject, 
    const Slic3r::Polygons &clip, const ClipperLib::PolyFillType fillType, bool safety_offset_ = false);

Slic3r::Polygons _clipper(ClipperLib::ClipType clipType,
    const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false);
Slic3r::ExPolygons _clipper_ex(ClipperLib::ClipType clipType,
    const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false);
Slic3r::Polylines _clipper_pl(ClipperLib::ClipType clipType,
    const Slic3r::Polylines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false);
Slic3r::Polylines _clipper_pl(ClipperLib::ClipType clipType,
    const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false);
Slic3r::Lines _clipper_ln(ClipperLib::ClipType clipType,
    const Slic3r::Lines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false);

// diff
inline Slic3r::Polygons
diff(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctDifference, subject, clip, safety_offset_);
}

inline Slic3r::ExPolygons
diff_ex(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctDifference, subject, clip, safety_offset_);
}

inline Slic3r::ExPolygons
diff_ex(const Slic3r::ExPolygons &subject, const Slic3r::ExPolygons &clip, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctDifference, to_polygons(subject), to_polygons(clip), safety_offset_);
}

inline Slic3r::Polygons
diff(const Slic3r::ExPolygons &subject, const Slic3r::ExPolygons &clip, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctDifference, to_polygons(subject), to_polygons(clip), safety_offset_);
}

inline Slic3r::Polylines
diff_pl(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_pl(ClipperLib::ctDifference, subject, clip, safety_offset_);
}

inline Slic3r::Polylines
diff_pl(const Slic3r::Polylines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_pl(ClipperLib::ctDifference, subject, clip, safety_offset_);
}

inline Slic3r::Lines
diff_ln(const Slic3r::Lines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_ln(ClipperLib::ctDifference, subject, clip, safety_offset_);
}

// intersection
inline Slic3r::Polygons
intersection(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctIntersection, subject, clip, safety_offset_);
}

inline Slic3r::ExPolygons
intersection_ex(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctIntersection, subject, clip, safety_offset_);
}

inline Slic3r::ExPolygons
intersection_ex(const Slic3r::ExPolygons &subject, const Slic3r::ExPolygons &clip, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctIntersection, to_polygons(subject), to_polygons(clip), safety_offset_);
}

inline Slic3r::Polygons
intersection(const Slic3r::ExPolygons &subject, const Slic3r::ExPolygons &clip, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctIntersection, to_polygons(subject), to_polygons(clip), safety_offset_);
}

inline Slic3r::Polylines
intersection_pl(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_pl(ClipperLib::ctIntersection, subject, clip, safety_offset_);
}

inline Slic3r::Polylines
intersection_pl(const Slic3r::Polylines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_pl(ClipperLib::ctIntersection, subject, clip, safety_offset_);
}

inline Slic3r::Lines
intersection_ln(const Slic3r::Lines &subject, const Slic3r::Polygons &clip, bool safety_offset_ = false)
{
    return _clipper_ln(ClipperLib::ctIntersection, subject, clip, safety_offset_);
}

// union
inline Slic3r::Polygons
union_(const Slic3r::Polygons &subject, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctUnion, subject, Slic3r::Polygons(), safety_offset_);
}

inline Slic3r::Polygons
union_(const Slic3r::Polygons &subject, const Slic3r::Polygons &subject2, bool safety_offset_ = false)
{
    return _clipper(ClipperLib::ctUnion, subject, subject2, safety_offset_);
}

inline Slic3r::ExPolygons
union_ex(const Slic3r::Polygons &subject, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctUnion, subject, Slic3r::Polygons(), safety_offset_);
}

inline Slic3r::ExPolygons
union_ex(const Slic3r::ExPolygons &subject, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctUnion, to_polygons(subject), Slic3r::Polygons(), safety_offset_);
}

inline Slic3r::ExPolygons
union_ex(const Slic3r::Surfaces &subject, bool safety_offset_ = false)
{
    return _clipper_ex(ClipperLib::ctUnion, to_polygons(subject), Slic3r::Polygons(), safety_offset_);
}


ClipperLib::PolyTree union_pt(const Slic3r::Polygons &subject, bool safety_offset_ = false);
Slic3r::Polygons union_pt_chained(const Slic3r::Polygons &subject, bool safety_offset_ = false);
void traverse_pt(ClipperLib::PolyNodes &nodes, Slic3r::Polygons* retval);

/* OTHER */
Slic3r::Polygons simplify_polygons(const Slic3r::Polygons &subject, bool preserve_collinear = false);
Slic3r::ExPolygons simplify_polygons_ex(const Slic3r::Polygons &subject, bool preserve_collinear = false);

void safety_offset(ClipperLib::Paths* paths);

}

#endif
