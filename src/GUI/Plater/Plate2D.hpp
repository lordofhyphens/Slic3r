#ifndef PLATE2D_HPP
#define PLATE2D_HPP
#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif
#include <wx/datetime.h>

#include <vector>
#include <functional>
#include "Plater.hpp"
#include "ColorScheme.hpp"
#include "Settings.hpp"
#include "Plater/PlaterObject.hpp"
#include "misc_ui.hpp"

#include "Log.hpp"


namespace Slic3r { namespace GUI {


// Setup for an Easter Egg with the canvas text.
const wxDateTime today_date {wxDateTime().GetDateOnly()}; 
const wxDateTime special_date {13, wxDateTime::Month::Sep, 2006, 0, 0, 0, 0};
const bool today_is_special = {today_date.GetDay() == special_date.GetDay() && today_date.GetMonth() == special_date.GetMonth()};

enum class MoveDirection {
    Up, Down, Left, Right
};

/// simple POD to make referencing this pair of identifiers easy
struct InstanceIdx {
    long obj;
    long inst;
};

class Plate2D : public wxPanel {
public:
    Plate2D(wxWindow* parent, const wxSize& size, std::vector<PlaterObject>& _objects, std::shared_ptr<Model> _model, std::shared_ptr<Config> _config, std::shared_ptr<Settings> _settings);


    /// Read print bed size from config and calculate the scaled rendition of the bed given the draw canvas.
    void update_bed_size();

    std::function<void (const unsigned int obj_idx)> on_select_object {};
    std::function<void ()> on_double_click {};

    /// Do something on right-clicks.
    std::function<void (const wxPoint& pos)> on_right_click {};
    std::function<void ()> on_instances_moved {};


    void set_selected (long obj, long inst) { this->selected_instance = {obj, inst}; }

private:
    std::vector<PlaterObject>& objects; //< reference to parent vector
    std::shared_ptr<Slic3r::Model> model;
    std::shared_ptr<Slic3r::Config> config;
    std::shared_ptr<Settings> settings;

    // Different brushes to draw with, initialized from settings->Color during the constructor
    wxBrush objects_brush {};
    wxBrush instance_brush {};
    wxBrush selected_brush {};
    wxBrush bed_brush {};
    wxBrush dragged_brush {};
    wxBrush transparent_brush {};

    wxPen grid_pen {};
    wxPen print_center_pen {};
    wxPen clearance_pen {};
    wxPen skirt_pen {};
    wxPen dark_pen {};

    bool user_drawn_background {(the_os == OS::Mac ? false : true)};

    /// The object id and selected 
    InstanceIdx selected_instance {-1, -1};
    InstanceIdx drag_object {-1, -1};

    /// Handle mouse-move events
    void mouse_drag(wxMouseEvent& e);
    void mouse_down(wxMouseEvent& e);
    void mouse_up(wxMouseEvent& e);
    void mouse_dclick(wxMouseEvent& e);

    wxPoint drag_start_pos {wxPoint(-1, -1)};


    /// Handle repaint events
    void repaint(wxPaintEvent& e);

    void nudge_key(wxKeyEvent& e);

    void nudge(MoveDirection dir);

    /// Set/Update all of the colors used by the various brushes in the panel.
    void set_colors();

    /// Convert a scale point array to a pixel polygon suitable for DrawPolygon
    std::vector<wxPoint> scaled_points_to_pixel(const Slic3r::Polygon& poly, bool unscale);
    std::vector<wxPoint> scaled_points_to_pixel(const Slic3r::Polyline& poly, bool unscale);

    /// For a specific point, unscale it relative to the origin
    wxPoint unscaled_point_to_pixel(const wxPoint& in);


    /// Displacement needed to center bed.
    wxPoint bed_origin {};

    /// private class variables to stash bits for drawing the print bed area.
    wxRealPoint print_center {};
    Slic3r::Polygon bed_polygon {};
    std::vector<wxPoint> grid {};

    /// Set up the 2D canvas blank canvas text. 
    /// Easter egg: Sept. 13, 2006. The first part ever printed by a RepRap to make another RepRap.
    const wxString CANVAS_TEXT { today_is_special ? _(L"What do you want to print today?™") : _("Drag your objects here") };

    /// How much to scale the points to fit in the draw bounding box area.
    /// Expressed as pixel / mm
    double scaling_factor {1.0};
    
    const std::string LogChannel {"GUI_2D"};

    Slic3r::Point point_to_model_units(coordf_t x, coordf_t y) {
        const auto& zero {this->bed_origin};
        return Slic3r::Point(
            scale_(x - zero.x) / this->scaling_factor,
            scale_(zero.y - y) / this->scaling_factor
        );
    }
    Slic3r::Point point_to_model_units(const wxPoint& pt) {
        return this->point_to_model_units(pt.x, pt.y);
    }
    Slic3r::Point point_to_model_units(const Pointf& pt) {
        return this->point_to_model_units(pt.x, pt.y);
    }

    /// Remove all instance thumbnails.
    void clean_instance_thumbnails();
};

} } // Namespace Slic3r::GUI
#endif // PLATE2D_HPP
