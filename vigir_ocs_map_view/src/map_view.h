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

namespace vigir_ocs
{
// Class "MapView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class MapView: public Base3DView
{
    Q_OBJECT
public:
    MapView( QWidget* parent = 0 );
    virtual ~MapView();
};
}
#endif // map_view_H
