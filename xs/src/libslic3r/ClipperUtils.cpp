#include "ClipperUtils.hpp"
#include "Geometry.hpp"

namespace Slic3r {

//-----------------------------------------------------------
// legacy code from Clipper documentation
void AddOuterPolyNodeToExPolygons(ClipperLib::PolyNode& polynode, ExPolygons* expolygons)
{  
  size_t cnt = expolygons->size();
  expolygons->resize(cnt + 1);
  (*expolygons)[cnt].contour = ClipperPath_to_Slic3rMultiPoint<Polygon>(polynode.Contour);
  (*expolygons)[cnt].holes.resize(polynode.ChildCount());
  for (int i = 0; i < polynode.ChildCount(); ++i)
  {
    (*expolygons)[cnt].holes[i] = ClipperPath_to_Slic3rMultiPoint<Polygon>(polynode.Childs[i]->Contour);
    //Add outer polygons contained by (nested within) holes ...
    for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++j)
      AddOuterPolyNodeToExPolygons(*polynode.Childs[i]->Childs[j], expolygons);
  }
}
 
ExPolygons
PolyTreeToExPolygons(ClipperLib::PolyTree& polytree)
{
    ExPolygons retval;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        AddOuterPolyNodeToExPolygons(*polytree.Childs[i], &retval);
    return retval;
}
//-----------------------------------------------------------

void
unscaleClipperPolygons(ClipperLib::Paths &polygons)
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

template <class T>
T
ClipperPath_to_Slic3rMultiPoint(const ClipperLib::Path &input)
{
    T retval;
    for (ClipperLib::Path::const_iterator pit = input.begin(); pit != input.end(); ++pit)
        retval.points.push_back(Point( (*pit).X, (*pit).Y ));
    return retval;
}
template Polygon ClipperPath_to_Slic3rMultiPoint<Polygon>(const ClipperLib::Path &input);

template <class T>
T
ClipperPaths_to_Slic3rMultiPoints(const ClipperLib::Paths &input)
{
    T retval;
    for (ClipperLib::Paths::const_iterator it = input.begin(); it != input.end(); ++it)
        retval.push_back(ClipperPath_to_Slic3rMultiPoint<typename T::value_type>(*it));
    return retval;
}

ExPolygons
ClipperPaths_to_Slic3rExPolygons(const ClipperLib::Paths &input)
{
    // init Clipper
    ClipperLib::Clipper clipper;
    clipper.Clear();
    
    // perform union
    clipper.AddPaths(input, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree polytree;
    clipper.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);  // offset results work with both EvenOdd and NonZero
    
    // write to ExPolygons object
    return PolyTreeToExPolygons(polytree);
}

ClipperLib::Path
Slic3rMultiPoint_to_ClipperPath(const MultiPoint &input)
{
    ClipperLib::Path retval;
    for (Points::const_iterator pit = input.points.begin(); pit != input.points.end(); ++pit)
        retval.push_back(ClipperLib::IntPoint( (*pit).x, (*pit).y ));
    return retval;
}

template <class T>
ClipperLib::Paths
Slic3rMultiPoints_to_ClipperPaths(const T &input)
{
    ClipperLib::Paths retval;
    for (typename T::const_iterator it = input.begin(); it != input.end(); ++it)
        retval.push_back(Slic3rMultiPoint_to_ClipperPath(*it));
    return retval;
}

void
scaleClipperPolygons(ClipperLib::Paths &polygons, const double scale)
{
    for (ClipperLib::Paths::iterator it = polygons.begin(); it != polygons.end(); ++it) {
        for (ClipperLib::Path::iterator pit = (*it).begin(); pit != (*it).end(); ++pit) {
            (*pit).X *= scale;
            (*pit).Y *= scale;
        }
    }
}

ClipperLib::Paths
_offset(const Polygons &polygons, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // read input
    ClipperLib::Paths input = Slic3rMultiPoints_to_ClipperPaths(polygons);
    
    // scale input
    scaleClipperPolygons(input, scale);
    
    // perform offset
    ClipperLib::ClipperOffset co;
    if (joinType == jtRound) {
        co.ArcTolerance = miterLimit;
    } else {
        co.MiterLimit = miterLimit;
    }
    co.AddPaths(input, joinType, ClipperLib::etClosedPolygon);
    ClipperLib::Paths retval;
    co.Execute(retval, (delta*scale));
    
    // unscale output
    scaleClipperPolygons(retval, 1/scale);
    return retval;
}

ClipperLib::Paths
_offset(const Polylines &polylines, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // read input
    ClipperLib::Paths input = Slic3rMultiPoints_to_ClipperPaths(polylines);
    
    // scale input
    scaleClipperPolygons(input, scale);
    
    // perform offset
    ClipperLib::ClipperOffset co;
    if (joinType == jtRound) {
        co.ArcTolerance = miterLimit;
    } else {
        co.MiterLimit = miterLimit;
    }
    co.AddPaths(input, joinType, ClipperLib::etOpenButt);
    ClipperLib::Paths retval;
    co.Execute(retval, (delta*scale));
    
    // unscale output
    scaleClipperPolygons(retval, 1/scale);
    return retval;
}

Polygons
offset(const Polygons &polygons, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // perform offset
    ClipperLib::Paths output = _offset(polygons, delta, scale, joinType, miterLimit);
    
    // convert into Polygons
    return ClipperPaths_to_Slic3rMultiPoints<Polygons>(output);
}

Polygons
offset(const Polylines &polylines, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // perform offset
    ClipperLib::Paths output = _offset(polylines, delta, scale, joinType, miterLimit);
    
    // convert into Polygons
    return ClipperPaths_to_Slic3rMultiPoints<Polygons>(output);
}

Surfaces
offset(const Surface &surface, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // perform offset
    ExPolygons expp = offset_ex(surface.expolygon, delta, scale, joinType, miterLimit);
    
    // clone the input surface for each expolygon we got
    Surfaces retval;
    retval.reserve(expp.size());
    for (ExPolygons::iterator it = expp.begin(); it != expp.end(); ++it) {
        Surface s = surface;  // clone
        s.expolygon = *it;
        retval.push_back(s);
    }
    return retval;
}

ExPolygons
offset_ex(const Polygons &polygons, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    // perform offset
    ClipperLib::Paths output = _offset(polygons, delta, scale, joinType, miterLimit);
    
    // convert into ExPolygons
    return ClipperPaths_to_Slic3rExPolygons(output);
}

ExPolygons
offset_ex(const ExPolygons &expolygons, const float delta,
    double scale, ClipperLib::JoinType joinType, double miterLimit)
{
    return offset_ex(to_polygons(expolygons), delta, scale, joinType, miterLimit);
}

ClipperLib::Paths
_offset2(const Polygons &polygons, const float delta1, const float delta2,
    const double scale, const ClipperLib::JoinType joinType, const double miterLimit)
{
    // read input
    ClipperLib::Paths input = Slic3rMultiPoints_to_ClipperPaths(polygons);
    
    // scale input
    scaleClipperPolygons(input, scale);
    
    // prepare ClipperOffset object
    ClipperLib::ClipperOffset co;
    if (joinType == jtRound) {
        co.ArcTolerance = miterLimit;
    } else {
        co.MiterLimit = miterLimit;
    }
    
    // perform first offset
    ClipperLib::Paths output1;
    co.AddPaths(input, joinType, ClipperLib::etClosedPolygon);
    co.Execute(output1, (delta1*scale));
    
    // perform second offset
    co.Clear();
    co.AddPaths(output1, joinType, ClipperLib::etClosedPolygon);
    ClipperLib::Paths retval;
    co.Execute(retval, (delta2*scale));
    
    // unscale output
    scaleClipperPolygons(retval, 1/scale);
    return retval;
}

Polygons
offset2(const Polygons &polygons, const float delta1, const float delta2,
    const double scale, const ClipperLib::JoinType joinType, const double miterLimit)
{
    // perform offset
    ClipperLib::Paths output = _offset2(polygons, delta1, delta2, scale, joinType, miterLimit);
    
    // convert into ExPolygons
    return ClipperPaths_to_Slic3rMultiPoints<Polygons>(output);
}

ExPolygons
offset2_ex(const Polygons &polygons, const float delta1, const float delta2,
    const double scale, const ClipperLib::JoinType joinType, const double miterLimit)
{
    // perform offset
    ClipperLib::Paths output = _offset2(polygons, delta1, delta2, scale, joinType, miterLimit);
    
    // convert into ExPolygons
    return ClipperPaths_to_Slic3rExPolygons(output);
}

template <class T>
T
_clipper_do(const ClipperLib::ClipType clipType, const Polygons &subject, 
    const Polygons &clip, const ClipperLib::PolyFillType fillType, const bool safety_offset_)
{
    // read input
    ClipperLib::Paths input_subject = Slic3rMultiPoints_to_ClipperPaths(subject);
    ClipperLib::Paths input_clip    = Slic3rMultiPoints_to_ClipperPaths(clip);
    
    // perform safety offset
    if (safety_offset_) {
        if (clipType == ClipperLib::ctUnion) {
            safety_offset(&input_subject);
        } else {
            safety_offset(&input_clip);
        }
    }
    
    // init Clipper
    ClipperLib::Clipper clipper;
    clipper.Clear();
    
    // add polygons
    clipper.AddPaths(input_subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(input_clip,    ClipperLib::ptClip,    true);
    
    // perform operation
    T retval;
    clipper.Execute(clipType, retval, fillType, fillType);
    return retval;
}

// The Clipper library has difficulties processing overlapping polygons.
// Namely, the function Clipper::JoinCommonEdges() has potentially a terrible time complexity if the output
// of the operation is of the PolyTree type.
// This function implements a following workaround:
// 1) Peform the Clipper operation with the output to Paths. This method handles overlaps in a reasonable time.
// 2) Run Clipper Union once again to extract the PolyTree from the result of 1).
inline ClipperLib::PolyTree _clipper_do_polytree2(const ClipperLib::ClipType clipType, const Polygons &subject, 
    const Polygons &clip, const ClipperLib::PolyFillType fillType, const bool safety_offset_)
{
    // read input
    ClipperLib::Paths input_subject = Slic3rMultiPoints_to_ClipperPaths(subject);
    ClipperLib::Paths input_clip    = Slic3rMultiPoints_to_ClipperPaths(clip);
    
    // perform safety offset
    if (safety_offset_) {
        if (clipType == ClipperLib::ctUnion) {
            safety_offset(&input_subject);
        } else {
            safety_offset(&input_clip);
        }
    }
    
    ClipperLib::Clipper clipper;
    clipper.AddPaths(input_subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(input_clip,    ClipperLib::ptClip,    true);
    // Perform the operation with the output to input_subject.
    // This pass does not generate a PolyTree, which is a very expensive operation with the current Clipper library
    // if there are overlapping edges.
    clipper.Execute(clipType, input_subject, fillType, fillType);
    // Perform an additional Union operation to generate the PolyTree ordering.
    clipper.Clear();
    clipper.AddPaths(input_subject, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree retval;
    clipper.Execute(ClipperLib::ctUnion, retval, fillType, fillType);
    return retval;
}

ClipperLib::PolyTree
_clipper_do(const ClipperLib::ClipType clipType, const Polylines &subject, 
    const Polygons &clip, const ClipperLib::PolyFillType fillType,
    const bool safety_offset_)
{
    // read input
    ClipperLib::Paths input_subject = Slic3rMultiPoints_to_ClipperPaths(subject);
    ClipperLib::Paths input_clip    = Slic3rMultiPoints_to_ClipperPaths(clip);
    
    // perform safety offset
    if (safety_offset_) safety_offset(&input_clip);
    
    // init Clipper
    ClipperLib::Clipper clipper;
    clipper.Clear();
    
    // add polygons
    clipper.AddPaths(input_subject, ClipperLib::ptSubject, false);
    clipper.AddPaths(input_clip,    ClipperLib::ptClip,    true);
    
    // perform operation
    ClipperLib::PolyTree retval;
    clipper.Execute(clipType, retval, fillType, fillType);
    return retval;
}

Polygons
_clipper(ClipperLib::ClipType clipType, const Polygons &subject, 
    const Polygons &clip, bool safety_offset_)
{
    // perform operation
    ClipperLib::Paths output = _clipper_do<ClipperLib::Paths>(clipType, subject, clip, ClipperLib::pftNonZero, safety_offset_);
    
    // convert into Polygons
    return ClipperPaths_to_Slic3rMultiPoints<Polygons>(output);
}

ExPolygons
_clipper_ex(ClipperLib::ClipType clipType, const Polygons &subject, 
    const Polygons &clip, bool safety_offset_)
{
    // perform operation
    ClipperLib::PolyTree polytree = _clipper_do_polytree2(clipType, subject, clip, ClipperLib::pftNonZero, safety_offset_);
    
    // convert into ExPolygons
    return PolyTreeToExPolygons(polytree);
}

Polylines
_clipper_pl(ClipperLib::ClipType clipType, const Polylines &subject, 
    const Polygons &clip, bool safety_offset_)
{
    // perform operation
    ClipperLib::PolyTree polytree = _clipper_do(clipType, subject, clip, ClipperLib::pftNonZero, safety_offset_);
    
    // convert into Polylines
    ClipperLib::Paths output;
    ClipperLib::PolyTreeToPaths(polytree, output);
    return ClipperPaths_to_Slic3rMultiPoints<Polylines>(output);
}

Polylines
_clipper_pl(ClipperLib::ClipType clipType, const Polygons &subject, 
    const Polygons &clip, bool safety_offset_)
{
    // transform input polygons into polylines
    Polylines polylines;
    polylines.reserve(subject.size());
    for (Polygons::const_iterator polygon = subject.begin(); polygon != subject.end(); ++polygon)
        polylines.push_back(*polygon);  // implicit call to split_at_first_point()
    
    // perform clipping
    Polylines retval = _clipper_pl(clipType, polylines, clip, safety_offset_);
    
    /* If the split_at_first_point() call above happens to split the polygon inside the clipping area
       we would get two consecutive polylines instead of a single one, so we go through them in order
       to recombine continuous polylines. */
    for (size_t i = 0; i < retval.size(); ++i) {
        for (size_t j = i+1; j < retval.size(); ++j) {
            if (retval[i].points.back().coincides_with(retval[j].points.front())) {
                /* If last point of i coincides with first point of j,
                   append points of j to i and delete j */
                retval[i].points.insert(retval[i].points.end(), retval[j].points.begin()+1, retval[j].points.end());
                retval.erase(retval.begin() + j);
                --j;
            } else if (retval[i].points.front().coincides_with(retval[j].points.back())) {
                /* If first point of i coincides with last point of j,
                   prepend points of j to i and delete j */
                retval[i].points.insert(retval[i].points.begin(), retval[j].points.begin(), retval[j].points.end()-1);
                retval.erase(retval.begin() + j);
                --j;
            } else if (retval[i].points.front().coincides_with(retval[j].points.front())) {
                /* Since Clipper does not preserve orientation of polylines, 
                   also check the case when first point of i coincides with first point of j. */
                retval[j].reverse();
                retval[i].points.insert(retval[i].points.begin(), retval[j].points.begin(), retval[j].points.end()-1);
                retval.erase(retval.begin() + j);
                --j;
            } else if (retval[i].points.back().coincides_with(retval[j].points.back())) {
                /* Since Clipper does not preserve orientation of polylines, 
                   also check the case when last point of i coincides with last point of j. */
                retval[j].reverse();
                retval[i].points.insert(retval[i].points.end(), retval[j].points.begin()+1, retval[j].points.end());
                retval.erase(retval.begin() + j);
                --j;
            }
        }
    }
    return retval;
}

Lines
_clipper_ln(ClipperLib::ClipType clipType, const Lines &subject, const Polygons &clip,
    bool safety_offset_)
{
    // convert Lines to Polylines
    Polylines polylines;
    polylines.reserve(subject.size());
    for (Lines::const_iterator line = subject.begin(); line != subject.end(); ++line)
        polylines.push_back(*line);
    
    // perform operation
    polylines = _clipper_pl(clipType, polylines, clip, safety_offset_);
    
    // convert Polylines to Lines
    Lines retval;
    for (Polylines::const_iterator polyline = polylines.begin(); polyline != polylines.end(); ++polyline)
        retval.push_back(*polyline);
    return retval;
}

ClipperLib::PolyTree
union_pt(const Polygons &subject, bool safety_offset_)
{
    return _clipper_do<ClipperLib::PolyTree>(ClipperLib::ctUnion, subject, Polygons(), ClipperLib::pftEvenOdd, safety_offset_);
}

Polygons
union_pt_chained(const Polygons &subject, bool safety_offset_)
{
    ClipperLib::PolyTree polytree = union_pt(subject, safety_offset_);
    
    Polygons retval;
    traverse_pt(polytree.Childs, &retval);
    return retval;
}

void
traverse_pt(ClipperLib::PolyNodes &nodes, Polygons* retval)
{
    /* use a nearest neighbor search to order these children
       TODO: supply start_near to chained_path() too? */
    
    // collect ordering points
    Points ordering_points;
    ordering_points.reserve(nodes.size());
    for (ClipperLib::PolyNodes::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Point p((*it)->Contour.front().X, (*it)->Contour.front().Y);
        ordering_points.push_back(p);
    }
    
    // perform the ordering
    ClipperLib::PolyNodes ordered_nodes;
    Slic3r::Geometry::chained_path_items(ordering_points, nodes, ordered_nodes);
    
    // push results recursively
    for (ClipperLib::PolyNodes::iterator it = ordered_nodes.begin(); it != ordered_nodes.end(); ++it) {
        // traverse the next depth
        traverse_pt((*it)->Childs, retval);
        
        Polygon p = ClipperPath_to_Slic3rMultiPoint<Polygon>((*it)->Contour);
        retval->push_back(p);
        if ((*it)->IsHole()) retval->back().reverse();  // ccw
    }
}

Polygons
simplify_polygons(const Polygons &subject, bool preserve_collinear)
{
    // convert into Clipper polygons
    ClipperLib::Paths input_subject = Slic3rMultiPoints_to_ClipperPaths(subject);
    
    ClipperLib::Paths output;
    if (preserve_collinear) {
        ClipperLib::Clipper c;
        c.PreserveCollinear(true);
        c.StrictlySimple(true);
        c.AddPaths(input_subject, ClipperLib::ptSubject, true);
        c.Execute(ClipperLib::ctUnion, output, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    } else {
        ClipperLib::SimplifyPolygons(input_subject, output, ClipperLib::pftNonZero);
    }
    
    // convert into Slic3r polygons
    return ClipperPaths_to_Slic3rMultiPoints<Polygons>(output);
}

ExPolygons
simplify_polygons_ex(const Polygons &subject, bool preserve_collinear)
{
    if (!preserve_collinear) {
        return union_ex(simplify_polygons(subject, preserve_collinear));
    }
    
    // convert into Clipper polygons
    ClipperLib::Paths input_subject = Slic3rMultiPoints_to_ClipperPaths(subject);
    
    ClipperLib::PolyTree polytree;
    
    ClipperLib::Clipper c;
    c.PreserveCollinear(true);
    c.StrictlySimple(true);
    c.AddPaths(input_subject, ClipperLib::ptSubject, true);
    c.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    
    // convert into ExPolygons
    return PolyTreeToExPolygons(polytree);
}

void safety_offset(ClipperLib::Paths* paths)
{
    // scale input
    scaleClipperPolygons(*paths, CLIPPER_OFFSET_SCALE);
    
    // perform offset (delta = scale 1e-05)
    ClipperLib::ClipperOffset co;
    co.MiterLimit = 2;
    co.AddPaths(*paths, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    co.Execute(*paths, 10.0 * CLIPPER_OFFSET_SCALE);
    
    // unscale output
    scaleClipperPolygons(*paths, 1.0/CLIPPER_OFFSET_SCALE);
}

}
