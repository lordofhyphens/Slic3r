#include "ArcFitting.hpp"
#include "libslic3r.h"
#include "Log.hpp"
#include <sstream>

namespace Slic3r {

std::string
ArcFitting::process_layer(const std::string &gcode)
{
    
    return gcode;
}

struct ArcProperties {
    double error, radius;
    bool cw;

};

ArcProperties compute_properties(Points::const_iterator start, Points::const_iterator end, Point center){
    double min = center.distance_to(*start);
    double max = min;
    Point p = *start;
    double arcdist = 0.0;
    start++;
    bool cw = center.ccw_angle(*start, *(start+1)) > PI;
    for(;start != end; start++){
        double dist = center.distance_to(*start);
        if(dist > max){
            max = dist;
        }
        if(dist < min){
            min = dist;
        }
        dist = p.distance_to(*start);
        if(dist > arcdist){
            arcdist = dist;
        }
        // switches direction, not an arc
        if((center.ccw_angle(*start, *(start+1)) > PI) != cw){
            return ArcProperties{INFINITY,0.0,false};
        }
        
    }
    //Log::warn("max_dist") << "Max: " << std::to_string(max) << " Min: " << std::to_string(min);
    return ArcProperties{(max-min)/2+min/arcdist*0.01,
                         (max+min)/2,cw};
}

std::vector<size_t> arc_detection(Polyline &pl, size_t N, double epsilon){
    if(pl.points.size() < N)return {};
    std::vector<size_t> out;
    Point center;
    auto length = 0U;
    auto end = pl.points.cbegin();
    //while(length < N){ end++; length++;}
    for(auto start = pl.points.cbegin(); start != pl.points.cend(); start++){
        double error = 0.0;
        while(error < epsilon && end != pl.points.cend()){
            end++;
            if(std::distance(start,end) >= N){
                center = Geometry::circle_taubin_newton(start,end);
                error = compute_properties(start,end,center).error;
            }
            // Point max diff check
        }
        if(std::distance(start,end) <= N){
            end = start+1;
        }
        out.push_back(std::distance(start,end-1));
    }
    return out;
}

// Simple greedy arc fit
std::vector<std::pair<size_t,size_t>> arc_fitting(std::vector<size_t> &arcs, Polyline &pl){
    std::vector<std::pair<size_t,size_t>> out;
    for(auto i = 0U; i < arcs.size(); i++){
        if(arcs.at(i) != 0){
            out.push_back(std::pair<size_t,size_t>(i,i+arcs.at(i)));
            i += arcs.at(i) - 1; // -1 to counter i++
        }
    }
    return out;
}

ArcPath build_arcpath(std::vector<std::pair<size_t,size_t>> &arcs, Polyline &pl){
    ArcPath out;
    //out.points = pl.points;
    auto i = 0U;
    for(auto arc : arcs){
        Log::warn("arc")  << arc.first << " , " << arc.second << std::endl;
        // TODO: assert arc.first && arc.second < pl.points.size
        for(; i < arc.first; i++){
            out.centers.push_back(Point(0,0));
            out.arc_types.push_back(LINE);
            out.points.push_back(pl.points.at(i));
        }
        Point center = Geometry::circle_taubin_newton(pl.points.cbegin()+arc.first,pl.points.cbegin()+arc.second);
        auto prop = compute_properties(pl.points.cbegin()+arc.first,pl.points.cbegin()+arc.second,center);
        Point scaled = pl.points.at(arc.first);
        scaled.scale((prop.radius*center.distance_to(scaled)));
        if(out.points.empty()){
            out.centers.push_back(Point(0,0));
            out.arc_types.push_back(LINE);
            out.points.push_back(scaled);
        }
        // Handle matching the start point up with arcs
        if(out.arc_types.back() != LINE){
            // TODO: math to match up arcs if needed
            if(scaled.distance_to(out.points.back()) < SCALED_EPSILON){
                out.points.back() = scaled;
            }else{
                out.centers.push_back(Point(0,0));
                out.arc_types.push_back(LINE);
                out.points.push_back(scaled);
            }
        }
        // If a line doesn't perfectly match up
        out.points.back() = scaled; 
        
        out.points.push_back(pl.points.at(arc.second));
        out.centers.push_back(center);        
        out.arc_types.push_back(prop.cw?CW:CCW);
        i = arc.second;
    }
    for(; i < pl.points.size(); i++){
        out.centers.push_back(Point(0,0));
        out.arc_types.push_back(LINE);
        out.points.push_back(pl.points.at(i));
    }
    return out;
}

ArcPath arcify(Polyline &pl, size_t N, double e){
    auto arcs = arc_detection(pl,N,e);
    auto final_arcs = arc_fitting(arcs,pl);
    return build_arcpath(final_arcs,pl);
}

}
