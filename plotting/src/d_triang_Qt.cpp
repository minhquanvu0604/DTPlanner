// #include <iostream>
// #include <vector>

// #include <QApplication>
// #include <QWidget>
// #include <QPainter>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/point_generators_2.h>
// #include <CGAL/Delaunay_triangulation_2.h>

// ///// TYPEDEFFEFEFEFEF RULES?
// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_2 Point_2;
// typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;

#include "d_triang_Qt.hpp"

PlotWidget::PlotWidget(
    const std::vector<Point_2>& points_global, 
    const std::vector<Point_2>& points_local, 
    const std::vector<std::pair<Point_2, Point_2>>& edges,
    const std::vector<std::pair<Point_2, Point_2>>& other_paths, 
    const std::vector<std::pair<Point_2, Point_2>>& best_path,
    const std::vector<std::pair<Point_2, Point_2>>& smooth_path,
    QWidget* parent
    )
    : QWidget{parent}, _points_global{points_global}, _points_local{points_local}, 
        _edges{edges}, _other_paths{other_paths}, _best_path{best_path}, _smooth_path{smooth_path} {}


void PlotWidget::paintEvent(QPaintEvent *) {

    set_up();

    // A QPainter object is typically created and used within the scope of a painting function like paintEvent. 
    // This is because QPainter is designed to be used only during painting operations, and its lifecycle is managed accordingly.
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);        
    const int vertice_R = 5;

    //Plot car
    painter.drawEllipse(_plot_location_pts.back(), vertice_R, vertice_R);
    _plot_location_pts.pop_back();


    // Plot cones
    for (int i = 0; i < _plot_location_pts.size(); i++){

        // Plot cone points
        painter.drawEllipse(_plot_location_pts.at(i), vertice_R, vertice_R);
        
        // Write local coordinates
        QString text = QString("(%1, %2)").arg(CGAL::to_double(_points_local.at(i).x())).arg(CGAL::to_double(_points_local.at(i).y()));
        // Measure the text size
        QFontMetrics metrics(painter.font());
        int textWidth = metrics.horizontalAdvance(text);
        int textHeight = metrics.height();
        // Adjust the text position to be above and centered over the point
        QPointF textPos_local = _plot_location_pts.at(i) - QPointF(textWidth / 2, textHeight + vertice_R);
        painter.drawText(textPos_local, text);        

        // Write global coordinates
        text = QString("(%1, %2)").arg(CGAL::to_double(_points_global.at(i).x())).arg(-CGAL::to_double(_points_global.at(i).y()));
        // Adjust the text position to be above and centered over the point
        QPointF textPos_global = _plot_location_pts.at(i) - QPointF(textWidth / 2, textHeight*2 + vertice_R);
        painter.drawText(textPos_global, text);
        }


    // Plot the edges
    for (const auto& edge : _plot_location_edges){
        painter.drawLine(edge.first, edge.second);
    }

    // Plot the other paths
    for (const auto& path_segment : _plot_location_other_paths){
        QPen pen;
        pen.setWidth(5); // width in pixels
        pen.setColor(QColor(255, 0, 0, 100)); 
        painter.setPen(pen);
        painter.drawLine(path_segment.first, path_segment.second);
    }

    // Plot the best path
    for (const auto& path_segment : _plot_location_best_path){
        QPen pen;
        pen.setWidth(10); // width in pixels
        pen.setColor(QColor(0, 255, 0, 100)); 
        painter.setPen(pen);
        painter.drawLine(path_segment.first, path_segment.second);
    }

    // Plot the smooth path
    for (const auto& path_segment : _plot_location_smooth_path){
        QPen pen;
        pen.setWidth(2); // width in pixels
        pen.setColor(QColor(0, 0, 255)); 
        painter.setPen(pen);
        painter.drawLine(path_segment.first, path_segment.second);
    }
}


Point_2 PlotWidget::change_frame(Point_2 pt_global_coord) {
    double new_x = 1000;
    double new_y = 1000;
    double new_yaw = M_PI/2;

    // Translate the point
    double translated_x = CGAL::to_double(pt_global_coord.x()) - new_x;
    double translated_y = CGAL::to_double(pt_global_coord.y()) - new_y;

    // Rotate the point
    double rotated_x = translated_x * cos(new_yaw) + translated_y * sin(new_yaw);
    double rotated_y = -translated_x * sin(new_yaw) + translated_y * cos(new_yaw);

    return Point_2(rotated_x, rotated_y);
}

 
void PlotWidget::update_bounds(const std::vector<Point_2>& tr_pts) {
    for (const auto pt : tr_pts) {
        _x_min = std::min(_x_min, CGAL::to_double(pt.x()));
        _x_max = std::max(_x_max, CGAL::to_double(pt.x()));
        _y_min = std::min(_y_min, CGAL::to_double(pt.y()));
        _y_max = std::max(_y_max, CGAL::to_double(pt.y()));
    }
    // Adjust bounds slightly for padding
    _x_min -= 10; _x_max += 10;
    _y_min -= 10; _y_max += 10;
    
    // Compute scale
    // NEEDS REVISION ------------------------- NNNNNNNNNN
    // std::cout << "height:  " << height() << std::endl;
    // Scale 
    if (_x_max - _x_min > _y_max - _y_min){
        _scale = width() / (_x_max - _x_min);
        // std::cout << "Scaled by width" << std::endl;
    }
    else {
        _scale = 900 / (_y_max - _y_min);
        // std::cout << "Scaled by height" << std::endl;
    }
}

void PlotWidget::set_up(){

    Point_2 car_pt(0,0);
    _points_local.push_back(car_pt);

    // Location to plot the points on Qt Windows
    std::vector<Point_2> tr_pts;
    for (const auto& point : _points_local){
        // Convert (x y) from Qt frame (right down) to robot frame (up left)
        // It is a conversion from global frame (initial robot frame) to local frame (Qt frame)
        auto tr_p = change_frame(point);
        double mirrored_x = -CGAL::to_double(tr_p.x()); // Mirror the x-coordinate
        double y = CGAL::to_double(tr_p.y());
        tr_pts.push_back(Point_2(mirrored_x, y));
    }
    
    update_bounds(tr_pts);
    
    // Convert cone points to Qt's frame and scale the widget window
    scale_points(tr_pts);

    // Convert the segments to Qt's frame
    _plot_location_edges = scale_segments(_edges);
    _plot_location_other_paths = scale_segments(_other_paths);
    _plot_location_best_path = scale_segments(_best_path);
    _plot_location_smooth_path = scale_segments(_smooth_path);
}


void PlotWidget::scale_points(const std::vector<Point_2>& tr_pts){
    
    // Scale points to fit within the axis limits
    for (int i = 0; i < tr_pts.size(); i++){            
        double x = (CGAL::to_double(tr_pts.at(i).x()) - _x_min) * _scale;
        double y = (CGAL::to_double(tr_pts.at(i).y()) - _y_min) * _scale;
        // std::cout << "Location plotted: x = " << x << " y = " << y << std::endl; 
        _plot_location_pts.push_back(QPointF(x,y));
    }
}


std::vector<std::pair<QPointF, QPointF>> PlotWidget::scale_segments(std::vector<std::pair<Point_2, Point_2>> segments){

    std::vector<std::pair<QPointF, QPointF>> plot_location_segments;
    // Calculate plot location for edges 
    for (const auto& edge : segments){
        // First point
        double mirrored_x1 = -CGAL::to_double(change_frame(edge.first).x());
        double y1 = CGAL::to_double(change_frame(edge.first).y());

        double x_1 = (mirrored_x1 - _x_min) * _scale;
        double y_1 = (y1 - _y_min) * _scale;            
        QPointF start(x_1, y_1);

        // Second point
        double mirrored_x2 = -CGAL::to_double(change_frame(edge.second).x());
        double y2 = CGAL::to_double(change_frame(edge.second).y());

        double x_2 = (mirrored_x2 - _x_min) * _scale;
        double y_2 = (y2 - _y_min) * _scale; 
        QPointF end(x_2, y_2);

        std::pair<QPointF, QPointF> edge_QPointF = std::make_pair(start, end);
        plot_location_segments.push_back(edge_QPointF);
    }
    return plot_location_segments;
}