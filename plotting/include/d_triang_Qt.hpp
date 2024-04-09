#ifndef D_TRIANG_QT_H
#define D_TRIANG_QT_H


#include <iostream>
#include <vector>

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

///// TYPEDEFFEFEFEFEF RULES?
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;



/*
A class for plotting out the path
Won't be used whatsoever when running real-time
*/
class PlotWidget : public QWidget {

public:

    PlotWidget(
        const std::vector<Point_2>& points_global, 
        const std::vector<Point_2>& points_local, 
        const std::vector<std::pair<Point_2, Point_2>>& edges,
        const std::vector<std::pair<Point_2, Point_2>>& other_paths, 
        const std::vector<std::pair<Point_2, Point_2>>& best_path,
        const std::vector<std::pair<Point_2, Point_2>>& smooth_path,
        QWidget* parent = nullptr
        );
        
private:

    void paintEvent(QPaintEvent *) override; 

    /*
    Convert (x y) from Qt frame (right down) to robot frame (up left)
    It is a conversion from global frame (initial robot frame) to local frame (Qt frame)
    
    change_frame() -> update_bounds() -> scale_points() -> scale_edges()
    */
    void set_up();

    /*
    Change frame of reference by 2D translation and rotation
    To be used in set_up()
     */
    Point_2 change_frame(Point_2 pt_global_coord);

    /*
    Set the min max of x and y to later make the window bounds all the points nicely
    Then compute the scale accordingly
    */    
    void update_bounds(const std::vector<Point_2>& tr_pts);

    std::vector<std::pair<QPointF, QPointF>> scale_segments(std::vector<std::pair<Point_2, Point_2>> segments);

    void scale_points(const std::vector<Point_2>& tr_pts);


    // Input geometries
    std::vector<Point_2> _points_global;
    std::vector<Point_2> _points_local;
    std::vector<std::pair<Point_2, Point_2>> _edges;
    std::vector<std::pair<Point_2, Point_2>> _other_paths;
    std::vector<std::pair<Point_2, Point_2>> _best_path;
    std::vector<std::pair<Point_2, Point_2>> _smooth_path;
    
    // Geometries ready to plot
    std::vector<QPointF> _plot_location_pts;
    std::vector<std::pair<QPointF, QPointF>> _plot_location_edges;
    std::vector<std::pair<QPointF, QPointF>> _plot_location_other_paths;
    std::vector<std::pair<QPointF, QPointF>> _plot_location_best_path;
    std::vector<std::pair<QPointF, QPointF>> _plot_location_smooth_path;
    

    double _x_min = std::numeric_limits<double>::max();
    double _x_max = std::numeric_limits<double>::lowest();
    double _y_min = _x_min;
    double _y_max = _x_max;

    double _scale = 1.0;
};

#endif