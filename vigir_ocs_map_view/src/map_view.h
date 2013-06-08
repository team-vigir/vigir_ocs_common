/*
 * MapView class definition.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials and the .
 *
 * Latest changes (12/08/2012):
 */

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>

#include "base_3d_view.h"

namespace rviz
{
class RenderPanelCustom;
}

namespace vigir_ocs
{
// Class "MapView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class MapView: public Base3DView
{
    Q_OBJECT
public:
    MapView( QWidget* parent = 0 );
    virtual ~MapView();

Q_SIGNALS:
    void queryPosition( int, int, Ogre::Vector3& );
    void unHighlight();

public Q_SLOTS:
    void enableSelectionTool(bool, int, int);
    void requestMap();

private:
    rviz::Tool* selection_tool_;

    ros::Publisher grid_map_request_pub_;

    int selected_area_[4];
};
}
#endif // map_view_H
