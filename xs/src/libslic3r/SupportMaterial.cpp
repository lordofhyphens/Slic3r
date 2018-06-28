#include "SupportMaterial.hpp"

namespace Slic3r
{

PrintObjectSupportMaterial::PrintObjectSupportMaterial(const PrintObject *object,
                                                       const PrintConfig *print_config,
                                                       const PrintObjectConfig *print_object_config,
                                                       const SupportParameters &support_params) :
    m_object(object),
    m_print_config(print_config),
    m_object_config(print_object_config),
    m_support_params(support_params),
    m_first_layer_flow(Flow()), // TODO
    m_support_material_flow(Flow()), // TODO @Samir55
    m_support_material_interface_flow(Flow()) // TODO @Samir55
{
    // Calculate a minimum support layer height as a minimum over all extruders, but not smaller than 10um.

    // Evaluate the XY gap between the object outer perimeters and the support structures.
}

/// TODO @Samir55
/// \param layer_storage
/// \param layer_type
/// \return
inline PrintObjectSupportMaterial::MyLayer &
layer_allocate(deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
               PrintObjectSupportMaterial::SupportLayerType layer_type)
{
    // Create a new Mylayer and enqueue it at the deque.
    layer_storage.emplace_back();

    // Add this new layer type.
    layer_storage.back().layer_type = layer_type;

    return layer_storage.back();
}

// TODO @Samir55 Add the thread safe allocator.
/*
 inline PrintObjectSupportMaterial::MyLayer& layer_allocate(
    std::deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
    tbb::spin_mutex                                 &layer_storage_mutex,
    PrintObjectSupportMaterial::SupporLayerType      layer_type)
{
    layer_storage_mutex.lock();
    layer_storage.push_back(PrintObjectSupportMaterial::MyLayer());
    PrintObjectSupportMaterial::MyLayer *layer_new = &layer_storage.back();
    layer_storage_mutex.unlock();
    layer_new->layer_type = layer_type;
    return *layer_new;
}
 */

/// Append MyLayers to a destination vector.
/// \param dst MyLayers destination vector.
/// \param src MyLayers source vector.
inline void
layers_append(PrintObjectSupportMaterial::MyLayersPtr &dst, const PrintObjectSupportMateriall::MyLayersPtr &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

/// Compare layers lexicographically.
struct MyLayersPtrCompare
{
    bool
    operator()(const PrintObjectSupportMaterial::MyLayer *layer1,
               const PrintObjectSupportMaterial::MyLayer *layer2) const
    {
        return *layer1 < *layer2;
    }
};

void
PrintObjectSupportMaterial::generate(PrintObject &object)
{

}

bool
PrintObjectSupportMaterial::has_raft() const
{
    return false;
}

bool
PrintObjectSupportMaterial::has_support() const
{
    return false;
}

bool
PrintObjectSupportMaterial::build_plate_only() const
{
    return false;
}

bool
PrintObjectSupportMaterial::synchronize_layers() const
{
    return false;
}

bool
PrintObjectSupportMaterial::has_contact_loops() const
{
    return false;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::top_contact_layers(const PrintObject &object,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas(const PrintObject &object,
                                                                          const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                          PrintObjectSupportMaterial::MyLayerStorage &layer_storage,
                                                                          std::vector<Polygons> &layer_support_areas) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 PrintObjectSupportMaterial::MyLayersPtr &top_contacts) const
{

}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::raft_and_intermediate_support_layers(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                 PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::generate_base_layers(const PrintObject &object,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                 PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                 const std::vector<Polygons> &layer_support_areas) const
{

}
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_raft_base(const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &base_layers,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_interface_layers(const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                      const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                      PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                      PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::trim_support_layers_by_object(const PrintObject &object,
                                                          PrintObjectSupportMaterial::MyLayersPtr &support_layers,
                                                          const coordf_t gap_extra_above,
                                                          const coordf_t gap_extra_below,
                                                          const coordf_t gap_xy) const
{

}

void
PrintObjectSupportMaterial::generate_toolpaths(const PrintObject &object,
                                               const PrintObjectSupportMaterial::MyLayersPtr &raft_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers) const
{

}

}