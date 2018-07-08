#ifndef SLIC3R_SUPPORTPARAMETES_H
#define SLIC3R_SUPPORTPARAMETES_H

#include <iostream>
#include <limits>
#include "PrintConfig.hpp"

using namespace std;
#define MIN_LAYER_HEIGHT 0.01
#define MIN_LAYER_HEIGHT_DEFAULT 0.07

namespace Slic3r
{
class SupportParameters
{
public:
    SupportParameters()
    { memset(this, 0, sizeof(SupportParameters)); }

    static SupportParameters
    create_from_config(
        const PrintConfig &print_config,
        const PrintObjectConfig &object_config,
        coordf_t object_height,
        const std::set<size_t> &object_extruders);

    // Has any raft layers?
    bool
    has_raft() const
    { return raft_layers() > 0; }

    size_t
    raft_layers() const
    { return base_raft_layers + interface_raft_layers; }

    // Is the 1st object layer height fixed, or could it be varied?
    bool
    first_object_layer_height_fixed() const
    { return !has_raft() || first_object_layer_bridging; }

    // Height of the object to be printed. This value does not contain the raft height.
    coordf_t
    object_print_z_height() const
    { return object_print_z_max - object_print_z_min; }

    // Number of raft layers.
    size_t base_raft_layers;
    // Number of interface layers including the contact layer.
    size_t interface_raft_layers;

    // Layer heights of the raft (base, interface and a contact layer).
    coordf_t base_raft_layer_height;
    coordf_t interface_raft_layer_height;
    coordf_t contact_raft_layer_height;
    bool contact_raft_layer_height_bridging;

    // The regular layer height, applied for all but the first layer, if not overridden by layer ranges
    // or by the variable layer thickness table.
    coordf_t layer_height;
    // Minimum / maximum layer height, to be used for the automatic adaptive layer height algorithm,
    // or by an interactive layer height editor.
    coordf_t min_layer_height;
    coordf_t max_layer_height;
    coordf_t max_suport_layer_height;

    // First layer height of the print, this may be used for the first layer of the raft
    // or for the first layer of the print.
    coordf_t first_print_layer_height;

    // Thickness of the first layer. This is either the first print layer thickness if printed without a raft,
    // or a bridging flow thickness if printed over a non-soluble raft,
    // or a normal layer height if printed over a soluble raft.
    coordf_t first_object_layer_height;

    // If the object is printed over a non-soluble raft, the first layer may be printed with a briding flow.
    bool first_object_layer_bridging;

    // Soluble interface? (PLA soluble in water, HIPS soluble in lemonen)
    // otherwise the interface must be broken off.
    bool soluble_interface;
    // Gap when placing object over raft.
    coordf_t gap_raft_object;
    // Gap when placing support over object.
    coordf_t gap_object_support;
    // Gap when placing object over support.
    coordf_t gap_support_object;

    // Bottom and top of the printed object.
    // If printed without a raft, object_print_z_min = 0 and object_print_z_max = object height.
    // Otherwise object_print_z_min is equal to the raft height.
    coordf_t raft_base_top_z;
    coordf_t raft_interface_top_z;
    coordf_t raft_contact_top_z;
    // In case of a soluble interface, object_print_z_min == raft_contact_top_z, otherwise there is a gap between the raft and the 1st object layer.
    coordf_t object_print_z_min;
    coordf_t object_print_z_max;
};
}
#endif //SLIC3R_SUPPORTPARAMETES_H
