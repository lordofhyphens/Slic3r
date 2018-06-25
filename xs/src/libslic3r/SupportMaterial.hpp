#ifndef slic3r_SupportMaterial_hpp_
#define slic3r_SupportMaterial_hpp_

namespace Slic3r {

// how much we extend support around the actual contact area
constexpr coordf_t SUPPORT_MATERIAL_MARGIN = 1.5;

class PrintObjectSupportMaterial
{
public:
    // Support layer type to be used by MyLayer. This type carries a much more detailed information
    // about the support layer type than the final support layers stored in a PrintObject.
    enum SupporLayerType
    {
        sltUnknown = 0,
        // Ratft base layer, to be printed with the support material.
        sltRaftBase,
        // Raft interface layer, to be printed with the support interface material.
        sltRaftInterface,
        // Bottom contact layer placed over a top surface of an object. To be printed with a support interface material.
        sltBottomContact,
        // Dense interface layer, to be printed with the support interface material.
        // This layer is separated from an object by an sltBottomContact layer.
        sltBottomInterface,
        // Sparse base support layer, to be printed with a support material.
        sltBase,
        // Dense interface layer, to be printed with the support interface material.
        // This layer is separated from an object with sltTopContact layer.
        sltTopInterface,
        // Top contact layer directly supporting an overhang. To be printed with a support interface material.
        sltTopContact,
        // Some undecided type yet. It will turn into sltBase first, then it may turn into sltBottomInterface or sltTopInterface.
        sltIntermediate,
    };

    // A support layer type used internally by the SupportMaterial class. This class carries a much more detailed
    // information about the support layer than the layers stored in the PrintObject, mainly
    // the MyLayer is aware of the bridging flow and the interface gaps between the object and the support.
    class MyLayer
    {};

    // Layers are allocated and owned by a deque. Once a layer is allocated, it is maintained
    // up to the end of a generate() method. The layer storage may be replaced by an allocator class in the future,
    // which would allocate layers by multiple chunks.
    typedef std::deque<MyLayer> MyLayerStorage;
    typedef std::vector<MyLayer *> MyLayersPtr;

public:
    // TODO Change to PrintConfig.
//    PrintObjectSupportMaterial(const PrintObject *object, const SlicingParameters &slicing_params);

    // Is raft enabled?
    bool has_raft() const;

    // Has any support?
    bool has_support() const;

    bool build_plate_only() const;

    bool synchronize_layers() const;

    bool has_contact_loops() const;

    // Generate support material for the object.
    // New support layers will be added to the object,
    // with extrusion paths and islands filled in for each support layer.
    void generate(PrintObject &object);

private:
    // Generate top contact layers supporting overhangs.
    // For a soluble interface material synchronize the layer heights with the object, otherwise leave the layer height undefined.
    // If supports over bed surface only are requested, don't generate contact layers over an object.
    MyLayersPtr top_contact_layers(const PrintObject &object, MyLayerStorage &layer_storage) const;

    // Generate bottom contact layers supporting the top contact layers.
    // For a soluble interface material synchronize the layer heights with the object,
    // otherwise set the layer height to a bridging flow of a support interface nozzle.
    MyLayersPtr bottom_contact_layers_and_layer_support_areas(
        const PrintObject &object, const MyLayersPtr &top_contacts, MyLayerStorage &layer_storage,
        std::vector<Polygons> &layer_support_areas) const;

    // Trim the top_contacts layers with the bottom_contacts layers if they overlap, so there would not be enough vertical space for both of them.
    void trim_top_contacts_by_bottom_contacts(const PrintObject &object, const MyLayersPtr &bottom_contacts,
                                              MyLayersPtr &top_contacts) const;

    // Generate raft layers and the intermediate support layers between the bottom contact and top contact surfaces.
    MyLayersPtr raft_and_intermediate_support_layers(
        const PrintObject &object,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayerStorage &layer_storage) const;

    // Fill in the base layers with polygons.
    void generate_base_layers(
        const PrintObject &object,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayersPtr &intermediate_layers,
        const std::vector<Polygons> &layer_support_areas) const;

    // Generate raft layers, also expand the 1st support layer
    // in case there is no raft layer to improve support adhesion.
    MyLayersPtr generate_raft_base(
        const MyLayersPtr &top_contacts,
        const MyLayersPtr &interface_layers,
        const MyLayersPtr &base_layers,
        MyLayerStorage &layer_storage) const;

    // Turn some of the base layers into interface layers.
    MyLayersPtr generate_interface_layers(
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayersPtr &intermediate_layers,
        MyLayerStorage &layer_storage) const;

    // Trim support layers by an object to leave a defined gap between
    // the support volume and the object.
    void trim_support_layers_by_object(
        const PrintObject &object,
        MyLayersPtr &support_layers,
        const coordf_t gap_extra_above,
        const coordf_t gap_extra_below,
        const coordf_t gap_xy) const;

/*
	void generate_pillars_shape();
	void clip_with_shape();
*/

    // Produce the actual G-code.
    void generate_toolpaths(
        const PrintObject &object,
        const MyLayersPtr &raft_layers,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        const MyLayersPtr &intermediate_layers,
        const MyLayersPtr &interface_layers) const;

    // Following objects are not owned by SupportMaterial class.
    const PrintObject *m_object;
    const PrintConfig *m_print_config;
    const PrintObjectConfig *m_object_config;

    // Pre-calculated parameters shared between the object slicer and the support generator,
    // carrying information on a raft, 1st layer height, 1st object layer height, gap between the raft and object etc.
    // TODO Change to PrintConfig.
//    SlicingParameters m_slicing_params;

    Flow m_first_layer_flow;
    Flow m_support_material_flow;
    Flow m_support_material_interface_flow;
    // Is merging of regions allowed? Could the interface & base support regions be printed with the same extruder?
    bool m_can_merge_support_regions;

    coordf_t m_support_layer_height_min;
    coordf_t m_support_layer_height_max;

    coordf_t m_gap_xy;
};

}

#endif
