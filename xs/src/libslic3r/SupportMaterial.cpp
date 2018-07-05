#include "SupportMaterial.hpp"

namespace Slic3r
{
/// TODO Check whether C++14 is gonna be used or not.
/// \param layer_storage
/// \param layer_storage_mutex
/// \param layer_type
/// \return
inline PrintObjectSupportMaterial::MyLayer &
layer_allocate(
    std::deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
    tbb::spin_mutex &layer_storage_mutex,
    PrintObjectSupportMaterial::SupportLayerType layer_type)
{
    layer_storage_mutex.lock();
    layer_storage.push_back(PrintObjectSupportMaterial::MyLayer());
    PrintObjectSupportMaterial::MyLayer *layer_new = &layer_storage.back();
    layer_storage_mutex.unlock();
    layer_new->layer_type = layer_type;
    return *layer_new;
}

/// Append MyLayers to a destination vector.
/// \param dst MyLayers destination vector.
/// \param src MyLayers source vector.
inline void
layers_append(PrintObjectSupportMaterial::MyLayersPtr &dst, const PrintObjectSupportMaterial::MyLayersPtr &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

/// Minimum layer height for the variable layer height algorithm.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t
min_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = print_config.min_layer_height.get_at(idx_nozzle - 1);
    return (min_layer_height == 0.0) ? MIN_LAYER_HEIGHT_DEFAULT : std::max(MIN_LAYER_HEIGHT, min_layer_height);
}

/// Maximum layer height for the variable layer height algorithm, 3/4 of a nozzle dimaeter by default,
/// it should not be smaller than the minimum layer height.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t
max_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = min_layer_height_from_nozzle(print_config, idx_nozzle);
    coordf_t max_layer_height = print_config.max_layer_height.get_at(idx_nozzle - 1);
    coordf_t nozzle_dmr = print_config.nozzle_diameter.get_at(idx_nozzle - 1);
    return std::max(min_layer_height, (max_layer_height == 0.) ? (0.75 * nozzle_dmr) : max_layer_height);
}

PrintObjectSupportMaterial::PrintObjectSupportMaterial(const PrintObject *object,
                                                       const SupportParameters &support_params) :
    m_object(object),
    m_print_config(&object->print()->config),
    m_object_config(&object->config),
    m_support_params(support_params),
    m_first_layer_flow(support_material_1st_layer_flow(object, float(support_params.first_print_layer_height))),
    m_support_material_flow(support_material_flow(object, float(support_params.layer_height))),
    m_support_material_interface_flow(support_material_interface_flow(object, float(support_params.layer_height)))
{
    // Calculate a minimum support layer height as a minimum over all extruders, but not smaller than 10um.
    m_support_layer_height_min = 1000000.0;
    for (auto lh : m_print_config->min_layer_height.values)
        m_support_layer_height_min = min(m_support_layer_height_min, max(0.1, lh));

    // if no interface layers allowed, print everything with the base support pattern.
    if (m_object_config->support_material_interface_layers.value == 0)
        m_support_material_interface_flow = m_support_material_flow;

    // TODO @Samir55 what's region ?
    // Evaluate the XY gap between the object outer perimeters and the support structures.
    coordf_t external_perimeter_width = 0.0;
    for (size_t region_id = 0; region_id < object->region_volumes.size(); ++region_id)
    {
        if (object->region_volumes.count(region_id) > 0 && !object->region_volumes.at(region_id).empty())
        {
            const PrintRegionConfig &config = object->print()->get_region(region_id)->config;
            coordf_t width = config.external_perimeter_extrusion_width.get_abs_value(m_support_params.layer_height);
            if (width <= 0.0)
                width = m_print_config->nozzle_diameter.get_at(config.perimeter_extruder - 1);
            external_perimeter_width = std::max(external_perimeter_width, width);
        }
    }
    m_gap_xy = m_object_config->support_material_xy_spacing.get_abs_value(external_perimeter_width);

    // TODO @Samir see later.
    m_can_merge_support_regions =
        m_object_config->support_material_extruder.value == m_object_config->support_material_interface_extruder.value;
    if (!m_can_merge_support_regions && (m_object_config->support_material_extruder.value == 0
        || m_object_config->support_material_interface_extruder.value == 0))
    {
        // One of the support extruders is of "don't care" type.
        auto object_extruders = m_object->print()->object_extruders();
        if (object_extruders.size() == 1 &&
            *object_extruders.begin() == std::max<unsigned int>(m_object_config->support_material_extruder.value,
                                                                m_object_config->support_material_interface_extruder.value))
            // Object is printed with the same extruder as the support.
            m_can_merge_support_regions = true;
    }
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

// TODO @Samir55
void
PrintObjectSupportMaterial::generate(PrintObject &object)
{

}

bool
PrintObjectSupportMaterial::has_raft() const
{
    return m_support_params.has_raft();
}

bool
PrintObjectSupportMaterial::has_support() const
{
    return m_object_config->support_material.value;
}

bool
PrintObjectSupportMaterial::build_plate_only() const
{
    return this->has_support() && m_object_config->support_material_buildplate_only.value;
}

bool
PrintObjectSupportMaterial::synchronize_layers() const
{
    return m_support_params.soluble_interface && m_object_config->support_material_synchronize_layers.value;
}

bool
PrintObjectSupportMaterial::has_contact_loops() const
{
    return m_object_config->support_material_interface_contact_loops.value;
}

// TODO @Samir55 move to Polygons
inline void polygons_append(Polygons &dst, Polygons &&src)
{
    if (dst.empty()) {
        dst = std::move(src);
    } else {
        std::move(std::begin(src), std::end(src), std::back_inserter(dst));
        src.clear();
    }
}

inline void polygons_append(Polygons &dst, const Polygons &src) { dst.insert(dst.end(), src.begin(), src.end()); }

// Collect outer contours of all slices of this layer.
// This is useful for calculating the support base with holes filled.
Polygons collect_slices_outer(const Layer &layer)
{
    Polygons out;
    out.reserve(out.size() + layer.slices.expolygons.size());
    for (ExPolygons::const_iterator it = layer.slices.expolygons.begin(); it != layer.slices.expolygons.end(); ++ it)
        out.push_back(it->contour);
    return out;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::top_contact_layers(const PrintObject &object,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    // Output layers, sorted by top Z.
    MyLayersPtr contact_out;

    // If user specified a custom angle threshold, convert it to radians.
    // Zero means automatic overhang detection.
    const double threshold_rad = (m_object_config->support_material_threshold.value > 0) ?
                                 M_PI * (m_object_config->support_material_threshold.value + 1) / 180. : // +1 makes the threshold inclusive
                                 0.; //TODO @Samir55 Check.


    // Build support on a build plate only? If so, then collect and union all the surfaces below the current layer.
    // Unfortunately this is an inherently a sequential process.
    const bool            build_plate_only = this->build_plate_only();
    std::vector<Polygons> build_plate_covered;
    if (build_plate_only) {
        build_plate_covered.assign(object.layers.size(), Polygons());
        for (size_t layer_id = 1; layer_id < object.layers.size(); ++ layer_id) {
            const Layer &lower_layer = *object.layers[layer_id-1];
            // Merge the new slices with the preceding slices.
            // Apply the safety offset to the newly added polygons, so they will connect
            // with the polygons collected before,
            // but don't apply the safety offset during the union operation as it would
            // inflate the polygons over and over. TODO @Samir55 Change this and see the differnece.
            Polygons &covered = build_plate_covered[layer_id];
            covered = build_plate_covered[layer_id - 1];
            polygons_append(covered, offset(lower_layer.slices.expolygons, scale_(0.01)));
            covered = union_(covered, false); // don't apply the safety offset.
        }
    }

    // Determine top contact areas.
    // If generating raft only (no support), only calculate top contact areas for the 0th layer.
    // If having a raft, start with 0th layer, otherwise with 1st layer.
    // Note that layer_id < layer->id when raft_layers > 0 as the layer->id incorporates the raft layers.
    // So layer_id == 0 means first object layer and layer->id == 0 means first print layer if there are no explicit raft layers.
    size_t num_layers = this->has_support() ? object.layer_count() : 1;
    tbb::spin_mutex layer_storage_mutex;
    tbb::parallel_for(tbb::blocked_range<size_t>(this->has_raft() ? 0 : 1, num_layers),
                      [this, &object, &build_plate_covered, threshold_rad, &layer_storage, &layer_storage_mutex, &contact_out](const tbb::blocked_range<size_t>& range) {
                          for (size_t layer_id = range.begin(); layer_id < range.end(); ++ layer_id) {
                              const Layer &layer = *object.layers[layer_id];

                              // Detect overhangs and contact areas needed to support them.
                              // Collect overhangs and contacts of all regions of this layer supported by the layer immediately below.
                              Polygons overhang_polygons;
                              Polygons contact_polygons;
                              Polygons slices_margin_cached;
                              float slices_margin_cached_offset = -1.0;

                              if (layer_id == 0) {
                                  // This is the first object layer, so the object is being printed on a raft and
                                  // we're here just to get the object footprint for the raft.
                                  // We only consider contours and discard holes to get a more continuous raft.
                                  overhang_polygons = collect_slices_outer(layer);
                                  // Extend by SUPPORT_MATERIAL_MARGIN, which is 1.5mm
                                  // TODO @Samir55 ask, this the maximum allowed area for supports infill?
                                  contact_polygons = offset(overhang_polygons, scale_(SUPPORT_MATERIAL_MARGIN), ClipperLib::jtMiter, 3);
                              } else {
                                  // Generate overhang / contact_polygons for non-raft layers.
                                  const Layer &lower_layer = *object.layers[layer_id-1];
                                  for (LayerRegion *layerm : layer.regions) {
                                      // Extrusion width accounts for the rounding of the extrudates.
                                      // It is the maximum width of the extrudate.
                                      float fw = float(layerm->flow(frExternalPerimeter).scaled_width());
                                      float lower_layer_offset =
                                          (layer_id < this->m_object_config->support_material_enforce_layers.value) ?
                                          // Enforce a full possible support, ignore the overhang angle.
                                          0.0f :
                                          (threshold_rad > 0.0 ?
                                           // Overhang defined by an angle.
                                           float(scale_(lower_layer.height / tan(threshold_rad))) :
                                           // Overhang defined by half the extrusion width.
                                           0.5f * fw);
                                      // Overhang polygons for this layer and region.
                                      Polygons diff_polygons;
                                      Polygons layerm_polygons = to_polygons(layerm->slices);
                                      Polygons lower_layer_polygons = to_polygons(lower_layer.slices.expolygons);
                                      if (lower_layer_offset == 0.f) {
                                          // Support everything (the difference between this layer polygons and the lower layer polygon.
                                          diff_polygons = diff(layerm_polygons, lower_layer_polygons);
                                          if (! build_plate_covered.empty()) {
                                              // Don't support overhangs above the top surfaces.
                                              // This step is done before the contact surface is calculated by growing the overhang region.
                                              diff_polygons = diff(diff_polygons, build_plate_covered[layer_id]);
                                          }
                                      } else {
                                          // Get the regions needing a support, collapse very tiny spots.
                                          //FIXME cache the lower layer offset if this layer has multiple regions.
                                          diff_polygons = offset2(
                                              diff(layerm_polygons,
                                                   offset(lower_layer_polygons, lower_layer_offset, SUPPORT_SURFACES_OFFSET_PARAMETERS)),
                                              -0.1f*fw, +0.1f*fw);
                                          if (! build_plate_covered.empty()) {
                                              // Don't support overhangs above the top surfaces.
                                              // This step is done before the contact surface is calculated by growing the overhang region.
                                              diff_polygons = diff(diff_polygons, build_plate_covered[layer_id]);
                                          }
                                          if (diff_polygons.empty())
                                              continue;
                                          // Offset the support regions back to a full overhang, restrict them to the full overhang.
                                          diff_polygons = diff(
                                              intersection(offset(diff_polygons, lower_layer_offset, SUPPORT_SURFACES_OFFSET_PARAMETERS), layerm_polygons),
                                              lower_layer_polygons);
                                      }
                                      if (diff_polygons.empty())
                                          continue;

                                      if (this->m_object_config->dont_support_bridges) {
                                          // compute the area of bridging perimeters
                                          // Note: this is duplicate code from GCode.pm, we need to refactor
//                                          if (true) { // TODO @Samir55
                                              Polygons bridged_perimeters;
                                              {
                                                  Flow bridge_flow = layerm->flow(frPerimeter, true);
                                                  coordf_t nozzle_diameter = m_print_config->nozzle_diameter.get_at(layerm->region()->config.perimeter_extruder-1);
                                                  Polygons lower_grown_slices = offset(lower_layer_polygons, 0.5f*float(scale_(nozzle_diameter)), SUPPORT_SURFACES_OFFSET_PARAMETERS);

                                                  // Collect perimeters of this layer.
                                                  // TODO: split_at_first_point() could split a bridge mid-way
                                                  Polylines overhang_perimeters;
                                                  for (ExtrusionEntity* extrusion_entity : layerm->perimeters.entities) {
                                                      const ExtrusionEntityCollection *island = dynamic_cast<ExtrusionEntityCollection*>(extrusion_entity);
                                                      assert(island != NULL);
                                                      for (size_t i = 0; i < island->entities.size(); ++ i) {
                                                          ExtrusionEntity *entity = island->entities[i];
                                                          ExtrusionLoop *loop = dynamic_cast<Slic3r::ExtrusionLoop*>(entity);
                                                          overhang_perimeters.push_back(loop ?
                                                                                        loop->as_polyline() :
                                                                                        dynamic_cast<const Slic3r::ExtrusionPath*>(entity)->polyline);
                                                      }
                                                  }

                                                  // workaround for Clipper bug, see Slic3r::Polygon::clip_as_polyline()
                                                  for (Polyline &polyline : overhang_perimeters)
                                                      polyline.points[0].x += 1;
                                                  // Trim the perimeters of this layer by the lower layer to get the unsupported pieces of perimeters.
                                                  overhang_perimeters = diff_pl(overhang_perimeters, lower_grown_slices);

                                                  // only consider straight overhangs
                                                  // only consider overhangs having endpoints inside layer's slices
                                                  // convert bridging polylines into polygons by inflating them with their thickness
                                                  // since we're dealing with bridges, we can't assume width is larger than spacing,
                                                  // so we take the largest value and also apply safety offset to be ensure no gaps
                                                  // are left in between
                                                  float w = float(std::max(bridge_flow.scaled_width(), bridge_flow.scaled_spacing()));
                                                  for (Polyline &polyline : overhang_perimeters)
                                                      if (polyline.is_straight()) {
                                                          // This is a bridge
                                                          polyline.extend_start(fw);
                                                          polyline.extend_end(fw);
                                                          // Is the straight perimeter segment supported at both sides?
                                                          if (layer.slices.contains(polyline.first_point()) && layer.slices.contains(polyline.last_point()))
                                                              // Offset a polyline into a thick line.
                                                              polygons_append(bridged_perimeters, offset(polyline, 0.5f * w + 10.f));
                                                      }
                                                  bridged_perimeters = union_(bridged_perimeters);
                                              }
                                              // remove the entire bridges and only support the unsupported edges
                                              Polygons bridges;
                                              for (const Surface &surface : layerm->fill_surfaces.surfaces)
                                                  if (surface.surface_type == stBottomBridge && surface.bridge_angle != -1)
                                                      polygons_append(bridges, surface.expolygon);
                                              diff_polygons = diff(diff_polygons, bridges, true);
                                              polygons_append(bridges, bridged_perimeters);
                                              polygons_append(diff_polygons,
                                                              intersection(
                                                                  // Offset unsupported edges into polygons.
                                                                  offset(layerm->unsupported_bridge_edges.polylines, scale_(SUPPORT_MATERIAL_MARGIN), ClipperLib::jtSquare, SUPPORT_SURFACES_OFFSET_PARAMETERS),
                                                                  bridges));
//                                          } else {
                                              // just remove bridged areas
//                                              diff_polygons = diff(diff_polygons, layerm->bridged, true);
//                                          }
                                      } // if (m_objconfig->dont_support_bridges)

                                      if (diff_polygons.empty())
                                          continue;

                                      if (this->has_contact_loops())
                                          polygons_append(overhang_polygons, diff_polygons);

                                      // Let's define the required contact area by using a max gap of half the upper
                                      // extrusion width and extending the area according to the configured margin.
                                      // We increment the area in steps because we don't want our support to overflow
                                      // on the other side of the object (if it's very thin).
                                      {
                                          //FIMXE 1) Make the offset configurable, 2) Make the Z span configurable.
                                          float slices_margin_offset = std::min(lower_layer_offset, float(scale_(m_gap_xy)));
                                          if (slices_margin_cached_offset != slices_margin_offset) {
                                              slices_margin_cached_offset = slices_margin_offset;
                                              slices_margin_cached = (slices_margin_offset == 0.f) ?
                                                                     to_polygons(lower_layer.slices.expolygons) :
                                                                     offset(lower_layer.slices.expolygons, slices_margin_offset, SUPPORT_SURFACES_OFFSET_PARAMETERS);
                                              if (! build_plate_covered.empty()) {
                                                  // Trim the inflated contact surfaces by the top surfaces as well.
                                                  polygons_append(slices_margin_cached, build_plate_covered[layer_id]);
                                                  slices_margin_cached = union_(slices_margin_cached);
                                              }
                                          }
                                          // Offset the contact polygons outside.
                                          for (size_t i = 0; i < NUM_MARGIN_STEPS; ++ i) {
                                              diff_polygons = diff(
                                                  offset(
                                                      diff_polygons,
                                                      SUPPORT_MATERIAL_MARGIN / NUM_MARGIN_STEPS,
                                                      ClipperLib::jtRound,
                                                      // round mitter limit
                                                      scale_(0.05)),
                                                  slices_margin_cached);
                                          }
                                      }
                                      polygons_append(contact_polygons, diff_polygons);
                                  } // for each layer.region
                              } // end of Generate overhang/contact_polygons for non-raft layers.

                              // now apply the contact areas to the layer were they need to be made
                              if (! contact_polygons.empty()) {
                                  // get the average nozzle diameter used on this layer
                                  MyLayer     &new_layer = layer_allocate(layer_storage, layer_storage_mutex, sltTopContact);
                                  new_layer.idx_object_layer_above = layer_id;
                                  if (m_support_params.soluble_interface) {
                                      // Align the contact surface height with a layer immediately below the supported layer.
                                      new_layer.print_z = layer.print_z - layer.height;
                                      if (layer_id == 0) {
                                          // This is a raft contact layer sitting directly on the print bed.
                                          new_layer.height   = m_support_params.contact_raft_layer_height;
                                          new_layer.bottom_z = m_support_params.raft_interface_top_z;
                                      } else {
                                          // Interface layer will be synchronized with the object.
                                          assert(layer_id > 0);
                                          new_layer.height = object.layers[layer_id - 1]->height;
                                          new_layer.bottom_z = (layer_id == 1) ? m_support_params.object_print_z_min : object.layers[layer_id - 2]->print_z;
                                      }
                                  } else {
                                      // Contact layer will be printed with a normal flow, but
                                      // it will support layers printed with a bridging flow.
                                      //FIXME Probably printing with the bridge flow? How about the unsupported perimeters? Are they printed with the bridging flow?
                                      // In the future we may switch to a normal extrusion flow for the supported bridges.
                                      // Get the average nozzle diameter used on this layer.
                                      coordf_t nozzle_dmr = 0.;
                                      for (const LayerRegion *region : layer.regions)
                                          nozzle_dmr += region->region()->nozzle_dmr_avg(*m_print_config);
                                      nozzle_dmr /= coordf_t(layer.regions.size());
                                      new_layer.print_z  = layer.print_z - nozzle_dmr - m_object_config->support_material_contact_distance;
                                      new_layer.bottom_z = new_layer.print_z;
                                      new_layer.height   = 0.;
                                      if (layer_id == 0) {
                                          // This is a raft contact layer sitting directly on the print bed.
                                          assert(this->has_raft());
                                          new_layer.bottom_z = m_support_params.raft_interface_top_z;
                                          new_layer.height   = m_support_params.contact_raft_layer_height;
                                      } else {
                                          // Ignore this contact area if it's too low.
                                          // Don't want to print a layer below the first layer height as it may not stick well.
                                          //FIXME there may be a need for a single layer support, then one may decide to print it either as a bottom contact or a top contact
                                          // and it may actually make sense to do it with a thinner layer than the first layer height.
                                          if (new_layer.print_z < m_support_params.first_print_layer_height - EPSILON) {
                                              // This contact layer is below the first layer height, therefore not printable. Don't support this surface.
                                              continue;
                                          } else if (new_layer.print_z < m_support_params.first_print_layer_height + EPSILON) {
                                              // Align the layer with the 1st layer height.
                                              new_layer.print_z  = m_support_params.first_print_layer_height;
                                              new_layer.bottom_z = 0;
                                              new_layer.height   = m_support_params.first_print_layer_height;
                                          } else {
                                              // Don't know the height of the top contact layer yet. The top contact layer is printed with a normal flow and
                                              // its height will be set adaptively later on.
                                          }
                                      }
                                  }

                                  SupportGridPattern support_grid_pattern(
                                      // Support islands, to be stretched into a grid.
                                      contact_polygons,
                                      // Trimming polygons, to trim the stretched support islands.
                                      slices_margin_cached,
                                      // How much to offset the extracted contour outside of the grid.
                                      m_object_config->support_material_spacing.value + m_support_material_flow.spacing(),
                                      Geometry::deg2rad(m_object_config->support_material_angle.value));
                                  // 1) infill polygons, expand them by half the extrusion width + a tiny bit of extra.
                                  new_layer.polygons = support_grid_pattern.extract_support(m_support_material_flow.scaled_spacing()/2 + 5);
                                  // 2) Contact polygons will be projected down. To keep the interface and base layers to grow, return a contour a tiny bit smaller than the grid cells.
                                  new_layer.contact_polygons = new Polygons(support_grid_pattern.extract_support(-3));

                                  // Even after the contact layer was expanded into a grid, some of the contact islands may be too tiny to be extruded.
                                  // Remove those tiny islands from new_layer.polygons and new_layer.contact_polygons.

                                  // Store the overhang polygons.
                                  // The overhang polygons are used in the path generator for planning of the contact loops.
                                  // if (this->has_contact_loops())
                                  new_layer.overhang_polygons = new Polygons(std::move(overhang_polygons));
                                  contact_out[layer_id] = &new_layer;
                              }
                          }
                      });

    // Compress contact_out, remove the nullptr items.
    remove_nulls(contact_out);

    return contact_out;
}

// TODO @Samir55
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas(const PrintObject &object,
                                                                          const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                          PrintObjectSupportMaterial::MyLayerStorage &layer_storage,
                                                                          std::vector<Polygons> &layer_support_areas) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

// TODO @Samir55
void
PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 PrintObjectSupportMaterial::MyLayersPtr &top_contacts) const
{

}

// TODO @Samir55
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::raft_and_intermediate_support_layers(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                 PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

// TODO @Samir55
void
PrintObjectSupportMaterial::generate_base_layers(const PrintObject &object,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                 PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                 const std::vector<Polygons> &layer_support_areas) const
{

}

// TODO @Samir55
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_raft_base(const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &base_layers,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

// TODO @Samir55
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_interface_layers(const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                      const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                      PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                      PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

// TODO @Samir55
void
PrintObjectSupportMaterial::trim_support_layers_by_object(const PrintObject &object,
                                                          PrintObjectSupportMaterial::MyLayersPtr &support_layers,
                                                          const coordf_t gap_extra_above,
                                                          const coordf_t gap_extra_below,
                                                          const coordf_t gap_xy) const
{

}

// TODO @Samir55
void
PrintObjectSupportMaterial::generate_toolpaths(const PrintObject &object,
                                               const PrintObjectSupportMaterial::MyLayersPtr &raft_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers) const
{

}

///
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

// TODO @Samir55 Refactor
Flow
support_material_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width.value > 0) ? object->config.support_material_extrusion_width
                                                                    : object->config.extrusion_width,
        // if object->config.support_material_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder - 1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

Flow
support_material_1st_layer_flow(const PrintObject *object, float layer_height)
{
    const auto &width = (object->print()->config.first_layer_extrusion_width.value > 0)
                        ? object->print()->config.first_layer_extrusion_width
                        : object->config.support_material_extrusion_width;
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (width.value > 0) ? width : object->config.extrusion_width,
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder - 1)),
        (layer_height > 0.0f) ? layer_height
                              : float(object->config.first_layer_height.get_abs_value(object->config.layer_height.value)),
        false);
}

Flow
support_material_interface_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterialInterface,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width > 0) ? object->config.support_material_extrusion_width
                                                              : object->config.extrusion_width,
        // if object->config.support_material_interface_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_interface_extruder - 1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

SupportParameters
SupportParameters::create_from_config(const PrintConfig &print_config,
                                      const PrintObjectConfig &object_config,
                                      coordf_t object_height,
                                      const std::set<size_t> &object_extruders)
{
    // Get the support extruder nozzle diameter.
    // TODO @Samir55 change this name.
    // If object_config.support_material_extruder == 0 resp. object_config.support_material_interface_extruder == 0,
    // which is consistent with the requirement that if support_material_extruder == 0 resp. support_material_interface_extruder == 0,
    // print_config.nozzle_diameter.get_at(size_t(-1)) returns the 0th nozzle diameter,
    // support will not trigger tool change, but it will use the current nozzle instead.
    // In that case all the nozzles have to be of the same diameter. // TODO @Samir55 This is intuitive?
    coordf_t support_material_extruder_dmr =
        print_config.nozzle_diameter.get_at(static_cast<size_t>(object_config.support_material_extruder.value - 1));

    coordf_t support_material_interface_extruder_dmr =
        print_config.nozzle_diameter.get_at(static_cast<size_t>(object_config.support_material_interface_extruder.value
            - 1));


    // Is the support of soluble type.
    bool soluble_interface = object_config.support_material_contact_distance.value == 0.0;

    // Get first layer height.
    coordf_t first_layer_height = (object_config.first_layer_height.value <= 0) ?
                                  object_config.layer_height.value :
                                  object_config.first_layer_height.get_abs_value(object_config.layer_height.value);

    // Create a support parameter object and return it.
    SupportParameters params;

    params.layer_height = object_config.layer_height.value;
    params.first_print_layer_height = first_layer_height;
    params.first_object_layer_height = first_layer_height;

    params.object_print_z_min = 0.0;
    params.object_print_z_max = object_height;

    params.base_raft_layers = static_cast<size_t>(object_config.raft_layers.value); // number of raft layers.
    params.soluble_interface = soluble_interface;

    // Minimum/maximum of the minimum layer height over all extruders.
    params.min_layer_height = MIN_LAYER_HEIGHT;
    params.max_layer_height = numeric_limits<double>::max();

    // Check if the print has some form of support. Add the support layers to the minimum / maximum layer height limits.
    if (object_config.support_material.value || params.base_raft_layers > 0)
    {
        params.min_layer_height = std::max(
            min_layer_height_from_nozzle(print_config, object_config.support_material_extruder),
            min_layer_height_from_nozzle(print_config, object_config.support_material_interface_extruder));

        params.max_layer_height = std::min(
            max_layer_height_from_nozzle(print_config, object_config.support_material_extruder),
            max_layer_height_from_nozzle(print_config, object_config.support_material_interface_extruder));

        params.max_suport_layer_height = params.max_layer_height;
    }

    if (object_extruders.empty())
    {
        params.min_layer_height = std::max(params.min_layer_height, min_layer_height_from_nozzle(print_config, 0));
        params.max_layer_height = std::min(params.max_layer_height, max_layer_height_from_nozzle(print_config, 0));
    }
    else
    {
        for (unsigned int extruder_id : object_extruders)
        {
            params.min_layer_height =
                std::max(params.min_layer_height, min_layer_height_from_nozzle(print_config, extruder_id));
            params.max_layer_height =
                std::min(params.max_layer_height, max_layer_height_from_nozzle(print_config, extruder_id));
        }
    }
    params.min_layer_height = std::min(params.min_layer_height, params.layer_height);
    params.max_layer_height = std::max(params.max_layer_height, params.layer_height);

    if (!soluble_interface)
    {
        params.gap_raft_object = object_config.support_material_contact_distance.value;
        params.gap_object_support = object_config.support_material_contact_distance.value;
        params.gap_support_object = object_config.support_material_contact_distance.value;
    }

    // TODO @Samir55 Check here.
    if (params.base_raft_layers > 0)
    {
        params.interface_raft_layers = (params.base_raft_layers + 1) / 2;
        params.base_raft_layers -= params.interface_raft_layers;

        // Use as large as possible layer height for the intermediate raft layers.
        params.base_raft_layer_height = std::max(params.layer_height, 0.75 * support_material_extruder_dmr);
        params.interface_raft_layer_height =
            std::max(params.layer_height, 0.75 * support_material_interface_extruder_dmr);
        params.contact_raft_layer_height_bridging = false;
        params.first_object_layer_bridging = false;

        params.contact_raft_layer_height =
            std::max(params.layer_height, 0.75 * support_material_interface_extruder_dmr);
        if (!soluble_interface)
        {
            // Compute the average of all nozzles used for printing the object over a raft.
            //FIXME It is expected, that the 1st layer of the object is printed with a bridging flow over a full raft. Shall it not be vice versa?
            coordf_t average_object_extruder_dmr = 0.;
            if (!object_extruders.empty())
            {
                for (unsigned int extruder_id : object_extruders)
                    average_object_extruder_dmr += print_config.nozzle_diameter.get_at(extruder_id);
                average_object_extruder_dmr /= coordf_t(object_extruders.size());
            }
            params.first_object_layer_height = average_object_extruder_dmr;
            params.first_object_layer_bridging = true;
        }
    }

    if (params.has_raft())
    {
        // Raise first object layer Z by the thickness of the raft itself plus the extra distance required by the support material logic.
        //FIXME The last raft layer is the contact layer, which shall be printed with a bridging flow for ease of separation. Currently it is not the case.
        if (params.raft_layers() == 1)
        {
            // There is only the contact layer.
            params.contact_raft_layer_height = first_layer_height;
            params.raft_contact_top_z = first_layer_height;
        }
        else
        {
            assert(params.base_raft_layers > 0);
            assert(params.interface_raft_layers > 0);
            // Number of the base raft layers is decreased by the first layer.
            params.raft_base_top_z =
                first_layer_height + coordf_t(params.base_raft_layers - 1) * params.base_raft_layer_height;
            // Number of the interface raft layers is decreased by the contact layer.
            params.raft_interface_top_z = params.raft_base_top_z
                + coordf_t(params.interface_raft_layers - 1) * params.interface_raft_layer_height;
            params.raft_contact_top_z = params.raft_interface_top_z + params.contact_raft_layer_height;
        }
        coordf_t print_z = params.raft_contact_top_z + params.gap_raft_object;
        params.object_print_z_min = print_z;
        params.object_print_z_max += print_z;
    }

    return params;
}

}