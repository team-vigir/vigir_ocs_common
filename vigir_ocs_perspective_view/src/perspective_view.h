/* 
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef PERSPECTIVE_VIEW_H
#define PERSPECTIVE_VIEW_H

#include <QWidget>

#include "base_3d_view.h"

namespace rviz
{
class VisualizationManager;
}

namespace vigir_ocs
{
// Class "PerspectiveView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class PerspectiveView: public Base3DView
{
    Q_OBJECT
public:
    PerspectiveView( QWidget* parent = 0, rviz::VisualizationManager* context = NULL );
    virtual ~PerspectiveView();
};
}
#endif // PERSPECTIVE_VIEW_H
